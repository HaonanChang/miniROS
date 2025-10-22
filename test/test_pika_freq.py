import os
import time
import numpy as np
from mini_ros.common.state import RobotAction
from mini_ros.test.frequency_test import MultThreadFreqTest, AsyncFreqTest
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from loguru import logger
import datetime
import sys


logger.remove()  # Remove default handler
logger.add(sys.stderr, level="ERROR")  # Only INFO and above will be shown


def create_timed_joint_cmds_traj(start_joint_point, end_joint_point, num_points, num_repeat):
    start_joint_cmds = np.linspace(start_joint_point, end_joint_point, num_points)
    end_joint_cmds = np.linspace(end_joint_point, start_joint_point, num_points)
    joint_cmds_traj = []
    for i in range(num_repeat):
        joint_cmds_traj += start_joint_cmds.tolist()
        joint_cmds_traj += end_joint_cmds.tolist()
    joint_cmds_traj = np.array(joint_cmds_traj)
    return joint_cmds_traj


def single_multi_thread_test(control_freq, read_freq, exp_idx=0, export_folder=""):
    # Read size = 1, 4, 0
    robot = PikaGripper(PikaGripperConfig(port="/dev/ttyUSB1", read_size=0))
    robot.start()
    robot.apply_action(RobotAction(timestamp=0, joint_cmds=[0]))
    time.sleep(1)

    # Generate joint traj
    joint_cmds_traj = create_timed_joint_cmds_traj(start_joint_point=np.array([0]), end_joint_point=np.array([0.5]), num_points=200, num_repeat=2)

    test = MultThreadFreqTest(robot, joint_cmds_traj=joint_cmds_traj, control_freq=control_freq, read_freq=read_freq)
    test.start()
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/c_{control_freq}-r_{read_freq}-e_{exp_idx}")
    robot.stop()


def batch_multi_thread_test():
    export_folder = f"./debug/{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(export_folder, exist_ok=True)
    for control_freq in [100]:
        for read_freq in [100]:
            for exp_idx in range(10):
                print(f"Testing c: {control_freq}, r: {read_freq}, exp_ids: {exp_idx}")
                single_multi_thread_test(control_freq=control_freq, read_freq=read_freq, exp_idx=exp_idx, export_folder=export_folder)


def single_async_test(control_freq, read_freq, exp_idx=0, export_folder=""):
    robot = PikaGripper(PikaGripperConfig(port="/dev/ttyUSB1", read_size=0))
    robot.start()
    robot.apply_action(RobotAction(timestamp=0, joint_cmds=[0]))
    time.sleep(1)

    # Generate joint traj
    joint_cmds_traj = create_timed_joint_cmds_traj(start_joint_point=np.array([0]), end_joint_point=np.array([0.5]), num_points=200, num_repeat=2)

    test = AsyncFreqTest(robot, joint_cmds_traj=joint_cmds_traj, control_freq=control_freq, read_freq=read_freq)
    test.start()
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/c_{control_freq}-r_{read_freq}-e_{exp_idx}")
    robot.stop()


def batch_async_test():
    export_folder = f"./debug/{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(export_folder, exist_ok=True)
    for control_freq in [100, 120, 140, 160, 180, 200]:
        for read_freq in [100, 120, 140, 160, 180, 200]:
            for exp_idx in range(1):
                print(f"Testing c: {control_freq}, r: {read_freq}, exp_ids: {exp_idx}")
                single_async_test(control_freq=control_freq, read_freq=read_freq, exp_idx=exp_idx, export_folder=export_folder)


if __name__ == "__main__":
    # batch_multi_thread_test()
    batch_async_test()