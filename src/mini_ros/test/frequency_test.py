"""
Ultilify functions for testing robot frequency.
"""
import matplotlib.pyplot as plt
import threading
import time
import asyncio
import numpy as np
from loguru import logger
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.common.device import Robot
from mini_ros.common.state import RobotAction, RobotState


class FreqTest:
    """
    Base class for frequency test.
    """
    def __init__(self, robot, joint_cmds_traj: list[list[float]], control_freq: int = 100, read_freq: int = 100):
        self.robot = robot
        self.control_freq = control_freq
        self.read_freq = read_freq
        self.joint_cmds_traj = joint_cmds_traj
        self._control_log: list[RobotAction] = []
        self._read_log: list[RobotState] = []

    def start(self):
        pass

    def join(self):
        pass   

    def control_loop(self):
        pass

    def read_loop(self):
        pass

    def generate_compare_fig(self, compare_type: str = "joint_cmds", title="") -> None:
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
            if title:
                plt.savefig(f"{title}_{compare_type}.png")
            else:
                plt.savefig(f"compare_{compare_type}.png")
        else:
            raise ValueError(f"Invalid compare type: {compare_type}")


class MultThreadFreqTest(FreqTest):
    """
    Test the robot read & write method into multi-thread.
    """
    def __init__(self, robot, joint_cmds_traj: list[list[float]], control_freq: int = 100, read_freq: int = 100):
        self.robot = robot
        self.control_freq = control_freq
        self.read_freq = read_freq
        self.control_thread = None
        self.read_thread = None
        self.joint_cmds_traj = joint_cmds_traj

        # Log
        self._control_log: list[RobotAction] = []
        self._read_log: list[RobotState] = []
    
    def start(self):
        self.control_thread = threading.Thread(target=self.control_loop)
        self.read_thread = threading.Thread(target=self.read_loop)
        self.control_thread.start()
        self.read_thread.start()

    def join(self):
        self.control_thread.join()
        self.read_thread.join()

    def control_loop(self) -> None:
        start_time = time.time()
        for joint_cmds in self.joint_cmds_traj:
            robot_action = RobotAction(timestamp=TimeUtil.now().timestamp(), joint_cmds=joint_cmds)
            self._control_log.append(robot_action)
            self.robot.apply_action(robot_action)
            time.sleep(1 / self.control_freq)
        logger.info("Quitting control loop.")
        # Set stop
        self.robot.stop()

    def read_loop(self) -> None:
        # start_time = time.time()
        while self.robot.is_active():
            try:
                state = self.robot.get_state()
            except Exception as e:
                break
            self._read_log.append(state)
            current_time = time.time()
            time.sleep(1/self.read_freq)
        logger.info("Quitting read loop.")

    def control_read_loop(self):
        # Call read & control sequentially
        for joint_cmds in self.joint_cmds_traj:
            robot_action = RobotAction(timestamp=TimeUtil.now().timestamp(), joint_cmds=joint_cmds)
            self.robot.apply_action(robot_action)
            self._control_log.append(robot_action)

            state = self.robot.get_state()
            self._read_log.append(state)
            
            # Rate
            time.sleep(1/self.control_freq)
            
        logger.info("Quitting read loop.")
        self.robot.stop()
    

class AsyncFreqTest(FreqTest):
    """
    Test the robot read & write method using async/await.
    """
    def __init__(self, robot, joint_cmds_traj: list[list[float]], control_freq: int = 100, read_freq: int = 100):
        self.robot = robot
        self.control_freq = control_freq
        self.read_freq = read_freq
        self.joint_cmds_traj = joint_cmds_traj
        
        # Log
        self._control_log: list[RobotAction] = []
        self._read_log: list[RobotState] = []
    
    def start(self):
        async def run_async_tasks():
            control_task = asyncio.create_task(self.control_loop())
            read_task = asyncio.create_task(self.read_loop())
            await asyncio.gather(control_task, read_task)
        
        AsyncUtil.run_async_as_blocking(run_async_tasks())

    async def control_loop(self):
        for joint_cmds in self.joint_cmds_traj:
            robot_action = RobotAction(timestamp=TimeUtil.now().timestamp(), joint_cmds=joint_cmds)
            self._control_log.append(robot_action)
            await AsyncUtil.run_blocking_as_async(self.robot.apply_action, robot_action)
            await TimeUtil.sleep_by_ms(1000/self.control_freq)
        logger.info("Quitting control loop.")
        # Set stop
        await AsyncUtil.run_blocking_as_async(self.robot.stop)

    async def read_loop(self):
        while self.robot.is_active():
            try:
                state = await AsyncUtil.run_blocking_as_async(self.robot.get_state)
            except Exception as e:
                break
            self._read_log.append(state)
            await TimeUtil.sleep_by_ms(1000/self.read_freq)
        logger.info("Quitting read loop.")