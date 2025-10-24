"""
Test the fixed trajectory for multi-robot.
"""
import os
import time
from typing import List
import threading
import numpy as np
from loguru import logger
from mini_ros.utils.rate_limiter import RateLimiterSync
from mini_ros.common.state import RobotAction
from mini_ros.system.robot_frequency_test import  create_timed_joint_cmds_traj, draw_compare_fig
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.io.recorder import EpisodeRecorder, RecorderConfig
from mini_ros.wrapper.multi_robot import MultiRobotSystem
from mini_ros.devices.cameras.rs_camera import RSCamera, RSCameraConfig
from mini_ros.wrapper.parallel_robot import ParallelRobotConfig, ParallelRobotMT
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.lang_util import LangUtil
from typing import Dict


phase_time = 2.0  # seconds

START_POSE = {
    "marvin": np.array([90.0, -70.0, -90.0, -110.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -110.0, -90.0, 0.0, 0.0]),
    "pika": np.array([0.0]),
}

END_POSE = {
    "marvin": np.array([90.0, -70.0, -90.0, -90.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -90.0, -90.0, 0.0, 0.0]),
    "pika": np.array([0.5]),
}


class FixedTrajRobotSys(ParallelRobotMT):

    def __init__(self, multi_robot: MultiRobotSystem, config: ParallelRobotConfig, joint_cmds_traj: Dict[str, list[list[float]]]):
        super().__init__(multi_robot, config)
        self.joint_cmds_traj = joint_cmds_traj
        # Sanity check
        for robot_name in self.multi_robot.robots.keys():
            if robot_name not in self.joint_cmds_traj.keys():
                raise ValueError(f"Robot {robot_name} not found in joint commands trajectory!")
            if len(self.joint_cmds_traj[robot_name]) == 0:
                raise ValueError(f"Joint commands trajectory for robot {robot_name} is empty!")

    def get_update_control_threads(self) -> List[threading.Thread]:
        update_control_threads = []
        for robot_name in self.multi_robot.robots.keys():
            update_control_threads.append(threading.Thread(target=self.control_update_loop, args=(robot_name,)))
        return update_control_threads

    def control_update_loop(self, robot_name: str) -> None:
        control_rate_limiter = RateLimiterSync(self.control_freqs[robot_name])
        step_idx = 0
        while True:
            if not self.is_alive():
                break
            if step_idx >= len(self.joint_cmds_traj[robot_name]):
                break
            try:
                control_rate_limiter.wait_for_tick()
                # Update action buffer
                with self.action_mutex[robot_name]:
                    self.action_buffer[robot_name] = RobotAction(joint_cmds=self.joint_cmds_traj[robot_name][step_idx])
                    step_idx += 1
                # logger.info(f"Updated action buffer for robot {robot_name} to step {step_idx}")
            except Exception as e:
                logger.error(f"Error in control update loop for robot {robot_name}: {type(e).__name__}: {e}")
                break

        # Trigger stop event
        self.stop_event.set()
        logger.info(f"Control update loop for robot {robot_name} stopped.")


if __name__ == "__main__":
    data_upload_dir = "./tmp_data"
    os.makedirs(data_upload_dir, exist_ok=True)
    
    robot_sys = MultiRobotSystem(
        devices=[
            RSCamera(RSCameraConfig(name="top_camera", device_id="341522302010", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            RSCamera(RSCameraConfig(name="left_camera", device_id="315122271722", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            RSCamera(RSCameraConfig(name="right_camera", device_id="315122271608", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            MarvinRobot(MarvinRobotConfig(name="marvin")),
            PikaGripper(PikaGripperConfig(name="pika-0", port="/dev/ttyUSB0", read_size=0)),
            PikaGripper(PikaGripperConfig(name="pika-1", port="/dev/ttyUSB1", read_size=0)),
            EpisodeRecorder(RecorderConfig(name="marvin", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
            EpisodeRecorder(RecorderConfig(name="pika-0", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
            EpisodeRecorder(RecorderConfig(name="pika-1", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
        ]
    )
    sys_config = ParallelRobotConfig(
        control_freqs={
            "marvin": 60,
            "pika-0": 60,
            "pika-1": 60,
        },
        read_freqs={
            "marvin": 200,
            "pika-0": 200,
            "pika-1": 200,
            "top_camera": 60,
            "left_camera": 60,
            "right_camera": 60,
        },
        episode_length=0,
    )
    joint_cmds_traj = {
        "marvin": create_timed_joint_cmds_traj(start_joint_point=START_POSE["marvin"], end_joint_point=END_POSE["marvin"], num_points=int(phase_time * sys_config.control_freqs["marvin"]), num_repeat=6),
        "pika-0": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * sys_config.control_freqs["pika-0"]), num_repeat=6),
        "pika-1": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * sys_config.control_freqs["pika-1"]), num_repeat=6),
    }

    # def update_config_callback():

    parallel_robot = FixedTrajRobotSys(robot_sys, sys_config, joint_cmds_traj)
    parallel_robot.initialize()
    for i in range(1):
        export_folder = os.path.join(data_upload_dir, f"e_{i:06d}")
        os.makedirs(export_folder, exist_ok=True)
        parallel_robot.start()
        parallel_robot.join()

    parallel_robot.stop()