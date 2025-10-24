"""
Test the for full robots: multiple robots and multiple cameras.
"""
import os
import time
import numpy as np
from mini_ros.common.state import RobotAction
from mini_ros.test.robot_frequency_test import MultiThreadFreqTestMultiRobot, create_timed_joint_cmds_traj
from mini_ros.test.robot_frequency_test import AsyncFreqTestMultiRobot
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.io.recorder import EpisodeRecorder, RecorderConfig
from mini_ros.wrapper.multi_robot import MultiRobotCamera
from mini_ros.devices.cameras.rs_camera import RSCamera, RSCameraConfig
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.lang_util import LangUtil
from loguru import logger
from typing import Dict
import datetime
import pathlib
from mini_ros.utils.video_util import VideoUtil
import sys
import asyncio


# logger.remove()  # Remove default handler
# logger.add(sys.stderr, level="ERROR")  # Only INFO and above will be shown

phase_time = 2.0  # seconds

START_POSE = {
    "marvin": np.array([90.0, -70.0, -90.0, -110.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -110.0, -90.0, 0.0, 0.0]),
    "pika": np.array([0.0]),
}

END_POSE = {
    "marvin": np.array([90.0, -70.0, -90.0, -90.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -90.0, -90.0, 0.0, 0.0]),
    "pika": np.array([0.5]),
}

# def move_tmp_file(goal_folder: str, camera: RSCamera):
#     target_path = pathlib.Path(goal_folder) / f"{camera.name}.jsonl"
#     logger.info(f"Moving {camera.tmp_file} to {target_path}")
#     # Move the file to the target folder
#     os.replace(camera.tmp_file, str(target_path))


def single_multi_thread_test(multi_robot: MultiRobotCamera, control_freqs: Dict[str, int], read_freqs: Dict[str, int], joint_cmds_traj: Dict[str, list[list[float]]], exp_idx=0, export_folder=""):
    if not multi_robot.is_active():
        multi_robot.start()
    for robot_name in multi_robot.robots.keys():
        multi_robot.apply_action_to(robot_name, RobotAction(timestamp=0, joint_cmds=joint_cmds_traj[robot_name][0]))
        time.sleep(1)

    control_freq_str = "_".join([f"{robot_name}:{control_freq}" for robot_name, control_freq in control_freqs.items()])
    read_freq_str = "_".join([f"{robot_name}:{read_freq}" for robot_name, read_freq in read_freqs.items()])

    test = MultiThreadFreqTestMultiRobot(multi_robot, control_freqs=control_freqs, read_freqs=read_freqs, joint_cmds_traj=joint_cmds_traj)
    test.start()
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/{control_freq_str}-{read_freq_str}-e_{exp_idx}", y_margin=10)
    # # Save video from cameras
    # for camera_name in multi_robot_camera.cameras.keys():
    #     move_tmp_file(export_folder, multi_robot_camera.cameras[camera_name])
    # Generate video
    # generate_video(export_folder)

def serial_test(multi_robot: MultiRobotCamera, control_freqs: Dict[str, int], read_freqs: Dict[str, int], joint_cmds_traj: Dict[str, list[list[float]]], exp_idx=0, export_folder=""):
    if not multi_robot.is_active():
        multi_robot.start()

    test = MultiThreadFreqTestMultiRobot(multi_robot, control_freqs=control_freqs, read_freqs=read_freqs, joint_cmds_traj=joint_cmds_traj)
    test.read_loop(device_name="top_camera", episode_length=2)


async def single_async_test(multi_robot: MultiRobotCamera, control_freqs: Dict[str, int], read_freqs: Dict[str, int], joint_cmds_traj: Dict[str, list[list[float]]], exp_idx=0, export_folder=""):
    if not multi_robot.is_active():
        multi_robot.start()

    for robot_name in multi_robot.robots.keys():
        multi_robot.apply_action_to(robot_name, RobotAction(timestamp=0, joint_cmds=joint_cmds_traj[robot_name][0]))
    time.sleep(1)

    control_freq_str = "_".join([f"{robot_name}:{control_freq}" for robot_name, control_freq in control_freqs.items()])
    read_freq_str = "_".join([f"{robot_name}:{read_freq}" for robot_name, read_freq in read_freqs.items()])
    # Generate joint traj
    test = AsyncFreqTestMultiRobot(multi_robot=multi_robot, joint_cmds_traj=joint_cmds_traj, control_freqs=control_freqs, read_freqs=read_freqs)
    test.start()

    # Wait for join
    await test.join()
    
    test.generate_compare_fig(title=f"{export_folder}/{control_freq_str}-{read_freq_str}-e_{exp_idx}", y_margin=10)
    # Save video from cameras
    # for camera_name in multi_robot.cameras.keys():
    #     move_tmp_file(export_folder, multi_robot.cameras[camera_name])
    # Generate video
    generate_video(export_folder)


def generate_video(base_path: str):
    # Run VA-API variant first
    start_time = TimeUtil.now()
    VideoUtil.generate_video(base_path, cpu_only=False)

    vaapi_ms = TimeUtil.get_elapsed_time_ms(start_time)

    logger.info(f"VA-API: {vaapi_ms:.2f} ms")

    # Run CPU variant
    start_time = TimeUtil.now()
    VideoUtil.generate_video(base_path, cpu_only=True)

    cpu_ms = TimeUtil.get_elapsed_time_ms(start_time)

    logger.info(f"CPU: {cpu_ms:.2f} ms")

    speedup = 1 / (vaapi_ms / cpu_ms)

    logger.info(f"VA-API: {vaapi_ms:.2f} ms, CPU: {cpu_ms:.2f} ms")
    logger.info(f"Speedup: {speedup:.2f}x")


if __name__ == "__main__":
    data_upload_dir = "./tmp_data"
    os.makedirs(data_upload_dir, exist_ok=True)
    
    multi_robot_camera = MultiRobotCamera(
        # robots={
        #     "marvin": MarvinRobot(MarvinRobotConfig()),
        #     "pika-0": PikaGripper(PikaGripperConfig(port="/dev/ttyUSB0", read_size=0)),
        #     "pika-1": PikaGripper(PikaGripperConfig(port="/dev/ttyUSB1", read_size=0)),
        # }, 
        # cameras={},
        cameras={
            "top_camera": RSCamera(RSCameraConfig(name="top_camera", device_id="341522302010", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            # "left_camera": RSCamera(RSCameraConfig(name="left_camera", device_id="315122271722", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            # "right_camera": RSCamera(RSCameraConfig(name="right_camera", device_id="315122271608", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
        },
        recorders={
            "marvin": EpisodeRecorder(RecorderConfig(name="marvin", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
            "pika-0": EpisodeRecorder(RecorderConfig(name="pika-0", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
            "pika-1": EpisodeRecorder(RecorderConfig(name="pika-1", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
        }
    )
    multi_robot_camera.initialize()
    multi_robot_camera.start()


    control_freqs = {
        "marvin": 60,
        "pika-0": 60,
        "pika-1": 60,
    }
    read_freqs = {
        "marvin": 200,
        "pika-0": 200,
        "pika-1": 200,
        "top_camera": 60,
        "left_camera": 60,
        "right_camera": 60,
    }
    joint_cmds_traj = {
        "marvin": create_timed_joint_cmds_traj(start_joint_point=START_POSE["marvin"], end_joint_point=END_POSE["marvin"], num_points=int(phase_time * control_freqs["marvin"]), num_repeat=6),
        "pika-0": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * control_freqs["pika-0"]), num_repeat=6),
        "pika-1": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * control_freqs["pika-1"]), num_repeat=6),
    }

    for i in range(1):
        export_folder = os.path.join(data_upload_dir, f"e_{i:06d}")
        os.makedirs(export_folder, exist_ok=True)
        # asyncio.run(single_async_test(multi_robot_camera, control_freqs, read_freqs, joint_cmds_traj, exp_idx=0, export_folder=export_folder))
        # single_multi_thread_test(multi_robot_camera, control_freqs, read_freqs, joint_cmds_traj, exp_idx=i, export_folder=export_folder)
        serial_test(multi_robot_camera, control_freqs, read_freqs, joint_cmds_traj, exp_idx=i, export_folder=export_folder)

    multi_robot_camera.stop()
