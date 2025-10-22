"""
Ultilify functions for testing robot frequency.
"""
import matplotlib.pyplot as plt
import threading
import time
import numpy as np
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.device import Robot
from mini_ros.common.state import RobotAction, RobotState


class MultThreadTest:
    """
    Test the robot read & write method into multi-thread.
    """
    def __init__(self, robot: Robot, joint_cmds_traj: list[list[float]], control_freq: int = 100, read_freq: int = 100):
        self.robot = robot
        self.control_freq = control_freq
        self.read_freq = read_freq
        self.control_thread = threading.Thread(target=self.control_loop)
        self.read_thread = threading.Thread(target=self.read_loop)
        self.control_thread.start()
        self.read_thread.start()
        self.joint_cmds_traj = joint_cmds_traj

        # Log
        self._control_log: list[RobotAction] = []
        self._read_log: list[RobotState] = []

    def control_loop(self) -> None:
        for joint_cmds in self.joint_cmds_traj:
            robot_action = RobotAction(timestamp=TimeUtil.now().timestamp(), joint_cmds=joint_cmds)
            self._control_log.append(robot_action)
            self.robot.apply_action(robot_action)
            current_time = time.time()
            if current_time - start_time >= 1 / self.control_freq:
                start_time = current_time
                self.robot.apply_action(RobotAction(timestamp=TimeUtil.now().timestamp(), joint_cmds=[0]))
            else:
                # Wait for the control interval
                time.sleep(0.001)

    def read_loop(self) -> None:
        start_time = time.time()
        while True:
            state = self.robot.get_state()
            self._read_log.append(state)
            current_time = time.time()
            if current_time - start_time >= 1 / self.read_freq:
                start_time = current_time
                self._read_log.append(state)
            else:
                # Wait for the read interval
                time.sleep(0.001)

    def generate_compare_fig(self, compare_type: str = "joint_cmds") -> None:
        """
        Generate the comparison figure of the control and read log.
        """
        if compare_type == "joint_cmds":
            control_timestamp = [action.timestamp for action in self._control_log]
            control_joint_cmds = [action.joint_cmds for action in self._control_log]
            control_joint_cmds = np.array(control_joint_cmds)
            control_joint_cmds -= control_joint_cmds[0]

            read_timestamp = [state.timestamp for state in self._read_log]
            read_joint_positions = [state.joint_positions for state in self._read_log]
            read_joint_positions = np.array(read_joint_positions)
            read_joint_positions -= read_joint_positions[0]

            plt.figure(figsize=(10, 5))
            plt.plot(control_timestamp, control_joint_cmds, label="Control")
            plt.plot(read_timestamp, read_joint_positions, label="Read")
            plt.legend()
            plt.savefig(f"compare_{compare_type}.png")
        else:
            raise ValueError(f"Invalid compare type: {compare_type}")