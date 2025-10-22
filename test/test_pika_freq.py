import numpy as np
from mini_ros.test.frequency_test import MultThreadTest
from mini_ros.devices.robots.pika_gripper import PikaGripper


if __name__ == "__main__":
    robot = PikaGripper()
    robot.initialize()
    robot.start()
    # Generate joint traj
    traj_1 = np.linspace(0, 0.095, 200)
    traj_2 = np.linspace(0.095, 0, 200)
    num_repeat = 10
    joint_cmds_traj = []
    for i in range(num_repeat):
        joint_cmds_traj += traj_1.tolist()  
        joint_cmds_traj += traj_2.tolist()

    test = MultThreadTest(robot, joint_cmds_traj=joint_cmds_traj, control_freq=100, read_freq=100)
    test.generate_compare_fig()
    robot.stop()