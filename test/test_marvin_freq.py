import os
import time
import numpy as np
from mini_ros.common.state import RobotAction
from mini_ros.test.frequency_test import MultThreadFreqTest, AsyncFreqTest, create_timed_joint_cmds_traj
# from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from loguru import logger
import datetime
import sys


# logger.remove()  # Remove default handler
# logger.add(sys.stderr, level="ERROR")  # Only INFO and above will be shown


marvin_start_pos = np.array([90.0, -70.0, -90.0, -110.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -110.0, -90.0, 0.0, 0.0])
marvin_end_pos = np.array([90.0, -70.0, -90.0, -90.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -90.0, -90.0, 0.0, 0.0])


def single_multi_thread_test(robot, control_freq, read_freq, exp_idx=0, export_folder=""):
    if not robot.is_active():
        robot.start()
    # Read size = 1, 4, 0
    robot.apply_action(RobotAction(timestamp=0, joint_cmds=marvin_start_pos))
    time.sleep(1)

    # Generate joint traj
    joint_cmds_traj = create_timed_joint_cmds_traj(start_joint_point=marvin_start_pos, end_joint_point=marvin_end_pos, num_points=200, num_repeat=2)

    test = MultThreadFreqTest(robot, joint_cmds_traj=joint_cmds_traj, control_freq=control_freq, read_freq=read_freq)
    test.start()
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/c_{control_freq}-r_{read_freq}-e_{exp_idx}", y_margin=10)


def batch_multi_thread_test(robot):
    export_folder = f"./debug/{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(export_folder, exist_ok=True)
    for control_freq in [100]:
        for read_freq in [100]:
            for exp_idx in range(2):
                print(f"Testing c: {control_freq}, r: {read_freq}, exp_ids: {exp_idx}")
                single_multi_thread_test(robot=robot, control_freq=control_freq, read_freq=read_freq, exp_idx=exp_idx, export_folder=export_folder)


def single_async_test(robot, control_freq, read_freq, exp_idx=0, export_folder=""):
    if not robot.is_active():
        robot.start()

    robot.apply_action(RobotAction(joint_cmds=marvin_start_pos))
    time.sleep(1)

    # Generate joint traj
    joint_cmds_traj = create_timed_joint_cmds_traj(start_joint_point=marvin_start_pos, end_joint_point=marvin_end_pos, num_points=200, num_repeat=2)

    test = AsyncFreqTest(robot, joint_cmds_traj=joint_cmds_traj, control_freq=control_freq, read_freq=read_freq)
    test.start()
    test.join()
    test.generate_compare_fig(title=f"{export_folder}/c_{control_freq}-r_{read_freq}-e_{exp_idx}", y_margin=10)


def batch_async_test(robot):
    export_folder = f"./debug/{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(export_folder, exist_ok=True)
    for control_freq in [100]:
        for read_freq in [100]:
            for exp_idx in range(2):
                print(f"Testing c: {control_freq}, r: {read_freq}, exp_ids: {exp_idx}")
                single_async_test(robot=robot, control_freq=control_freq, read_freq=read_freq, exp_idx=exp_idx, export_folder=export_folder)


if __name__ == "__main__":
    robot = MarvinRobot(MarvinRobotConfig())
    robot.initialize()
    robot.start()

    # # batch_multi_thread_test(robot)
    batch_async_test(robot)
    
    robot.stop()
    # robot.stop()