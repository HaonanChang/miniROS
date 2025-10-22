import time
import numpy as np
from mini_ros.common.state import RobotAction
from mini_ros.test.frequency_test import MultThreadTest
from mini_ros.devices.robots.pika_gripper_v2 import PikaGripper, PikaGripperConfig


if __name__ == "__main__":
    robot = PikaGripper(PikaGripperConfig(port="/dev/ttyUSB0"))
    robot.initialize(None)
    robot.start()
    robot.apply_action(RobotAction(timestamp=0, joint_cmds=[0]))
    time.sleep(1)

    # Generate joint traj
    traj_1 = np.linspace(0, 0.5, 600)
    traj_2 = np.linspace(0.5, 0, 600)
    num_repeat = 2
    joint_cmds_traj = []
    for i in range(num_repeat):
        joint_cmds_traj += traj_1.tolist()  
        joint_cmds_traj += traj_2.tolist()
    joint_cmds_traj = np.array(joint_cmds_traj).reshape(-1, 1)

    test = MultThreadTest(robot, joint_cmds_traj=joint_cmds_traj, control_freq=100, read_freq=100)
    test.start()
    test.join()
    test.generate_compare_fig()
    robot.stop()