"""
Test the for multiple robots.
"""
import os
import time
import numpy as np
from mini_ros.common.state import RobotAction
from mini_ros.test.frequency_test import MultiThreadFreqTestMultiRobot, AsyncFreqTestMultiRobot, AsyncFreqTest, create_timed_joint_cmds_traj
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.wrapper.multi_robot import MultiRobot
from loguru import logger
from typing import Dict
import datetime
import sys


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


def single_multi_thread_test(multi_robot: MultiRobot, control_freqs: Dict[str, int], read_freqs: Dict[str, int], joint_cmds_traj: Dict[str, list[list[float]]], exp_idx=0, export_folder=""):
    if not multi_robot.is_active():
        multi_robot.start()
    # Read size = 1, 4, 0
    for robot_name in multi_robot.robots.keys():
        multi_robot.apply_action_to(robot_name, RobotAction(timestamp=0, joint_cmds=joint_cmds_traj[robot_name][0]))
        time.sleep(1)

    control_freq_str = "_".join([f"{robot_name}:{control_freq}" for robot_name, control_freq in control_freqs.items()])
    read_freq_str = "_".join([f"{robot_name}:{read_freq}" for robot_name, read_freq in read_freqs.items()])

    test = MultiThreadFreqTestMultiRobot(multi_robot, control_freqs=control_freqs, read_freqs=read_freqs, joint_cmds_traj=joint_cmds_traj)
    test.start()
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/{control_freq_str}-{read_freq_str}-e_{exp_idx}", y_margin=10)


def single_async_test(multi_robot: MultiRobot, control_freqs: Dict[str, int], read_freqs: Dict[str, int], joint_cmds_traj: Dict[str, list[list[float]]], exp_idx=0, export_folder=""):
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
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/{control_freq_str}-{read_freq_str}-e_{exp_idx}", y_margin=10)


if __name__ == "__main__":
    marvin_robot = MarvinRobot(MarvinRobotConfig())
    pika_robot_0 = PikaGripper(PikaGripperConfig(port="/dev/ttyUSB0", read_size=0))
    pika_robot_1 = PikaGripper(PikaGripperConfig(port="/dev/ttyUSB1", read_size=0))
    multi_robot = MultiRobot(
        robots={"marvin": marvin_robot, "pika-0": pika_robot_0, "pika-1": pika_robot_1}, 
        robot_configs={
            "marvin": MarvinRobotConfig(), 
            "pika-0": PikaGripperConfig(port="/dev/ttyUSB0", read_size=0),
            "pika-1": PikaGripperConfig(port="/dev/ttyUSB1", read_size=0),
        }
    )
    multi_robot.initialize()
    multi_robot.start()


    control_freqs = {
        "marvin": 200,
        "pika-0": 200,
        "pika-1": 200,
    }
    read_freqs = {
        "marvin": 200,
        "pika-0": 200,
        "pika-1": 200,
    }
    joint_cmds_traj = {
        "marvin": create_timed_joint_cmds_traj(start_joint_point=START_POSE["marvin"], end_joint_point=END_POSE["marvin"], num_points=int(phase_time * control_freqs["marvin"]), num_repeat=2),
        "pika-0": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * control_freqs["pika-0"]), num_repeat=2),
        "pika-1": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * control_freqs["pika-1"]), num_repeat=2),
    }

    export_folder = f"./debug/{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(export_folder, exist_ok=True)
    # single_multi_thread_test(multi_robot, control_freqs, read_freqs, joint_cmds_traj, exp_idx=0, export_folder=export_folder)
    single_async_test(multi_robot, control_freqs, read_freqs, joint_cmds_traj, exp_idx=0, export_folder=export_folder)
    multi_robot.stop()
