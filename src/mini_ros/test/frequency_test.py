"""
Ultilify functions for testing robot frequency.
"""
import matplotlib.pyplot as plt
import threading
import time
import asyncio
import numpy as np
from typing import Union
from loguru import logger
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.common.device import Robot
from mini_ros.common.state import RobotAction, RobotState
from mini_ros.utils.rate_limiter import RateLimiter, RateLimiterSync


def create_timed_joint_cmds_traj(start_joint_point, end_joint_point, num_points, num_repeat):
    """
    Create a trajectory of joint commands.
    
    Args:
        start_joint_point: Starting joint positions, can be 1D (N,) or 2D (N, M)
        end_joint_point: Ending joint positions, can be 1D (N,) or 2D (N, M)
        num_points: Number of points in the trajectory
        num_repeat: Number of times to repeat the trajectory
    
    Returns:
        joint_cmds_traj: Array of shape (num_points * 2 * num_repeat, N) or (num_points * 2 * num_repeat, N, M)
    """
    # Convert to numpy arrays if they aren't already
    start_joint_point = np.array(start_joint_point)
    end_joint_point = np.array(end_joint_point)
    
    start_joint_cmds = np.linspace(start_joint_point, end_joint_point, num_points)
    end_joint_cmds = np.linspace(end_joint_point, start_joint_point, num_points)
    
    joint_cmds_traj = []
    for i in range(num_repeat):
        joint_cmds_traj += start_joint_cmds.tolist()
        joint_cmds_traj += end_joint_cmds.tolist()
    joint_cmds_traj = np.array(joint_cmds_traj)
    return joint_cmds_traj


class FreqTest:
    """
    Base class for frequency test.
    """
    def __init__(self, robot, joint_cmds_traj: Union[list[list[float]], list[list[list[float]]]], control_freq: int = 100, read_freq: int = 100):
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

    def generate_compare_fig(self, compare_type: str = "joint_cmds", title="", y_margin=0.0) -> None:
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

            # Align timestamp
            time_0 = control_timestamp[0]
            control_timestamp = [timestamp - time_0 for timestamp in control_timestamp]
            read_timestamp = [timestamp - time_0 for timestamp in read_timestamp]

            # Handle both 1D and 2D cases
            if control_joint_cmds.ndim == 2 and control_joint_cmds.shape[1] > 1:
                # 2D case: create subplots for each joint
                num_joints = control_joint_cmds.shape[1]
                grid_size = int(np.ceil(np.sqrt(num_joints)))
                
                fig, axes = plt.subplots(grid_size, grid_size, figsize=(15, 15))
                axes = axes.flatten() if num_joints > 1 else [axes]
                
                for i in range(num_joints):
                    ax = axes[i]
                    ax.plot(control_timestamp, control_joint_cmds[:, i], label=f"Control Joint {i}")
                    if read_joint_positions.ndim == 2 and read_joint_positions.shape[1] > i:
                        ax.plot(read_timestamp, read_joint_positions[:, i], label=f"Read Joint {i}")
                    ax.set_title(f"Joint {i}")
                    ax.legend()
                    ax.grid(True)
                    # Set limits
                    y_min, y_max = ax.get_ylim()
                    ax.set_ylim(y_min - y_margin, y_max + y_margin)
                
                # Hide unused subplots
                for i in range(num_joints, len(axes)):
                    axes[i].set_visible(False)
                    
            else:
                # 1D case: single subplot
                plt.figure(figsize=(10, 5))
                plt.plot(control_timestamp, control_joint_cmds, label="Control")
                plt.plot(read_timestamp, read_joint_positions, label="Read")
                plt.legend()
                plt.grid(True)
            
            # Calculate frequency
            control_freq = 1 / np.mean(np.diff(control_timestamp))
            read_freq = 1 / np.mean(np.diff(read_timestamp))
            logger.info(f"Control frequency: {control_freq}, Read frequency: {read_freq}")
            
            # Add frequency text to the plot with better visibility (split into two lines)
            control_text = f"Control freq: {control_freq:.1f} Hz"
            read_text = f"Read freq: {read_freq:.1f} Hz"
            
            if control_joint_cmds.ndim == 2 and control_joint_cmds.shape[1] > 1:
                # For subplots, add text to the first subplot
                axes[0].text(0.02, 0.98, control_text, transform=axes[0].transAxes, 
                           verticalalignment='top', horizontalalignment='left',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                           fontsize=10, fontweight='bold')
                axes[1].text(0.02, 0.98, read_text, transform=axes[1].transAxes, 
                           verticalalignment='top', horizontalalignment='left',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                           fontsize=10, fontweight='bold')
            else:
                # For single plot
                plt.text(0.02, 0.98, control_text, transform=plt.gca().transAxes, 
                        verticalalignment='top', horizontalalignment='left',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                        fontsize=10, fontweight='bold')
                plt.text(0.02, 0.90, read_text, transform=plt.gca().transAxes, 
                        verticalalignment='top', horizontalalignment='left',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                        fontsize=10, fontweight='bold')

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
    def __init__(self, robot, joint_cmds_traj: Union[list[list[float]], list[list[list[float]]]], control_freq: int = 100, read_freq: int = 100):
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
        control_rate_limiter = RateLimiterSync(self.control_freq)
        for joint_cmds in self.joint_cmds_traj:
            control_rate_limiter.wait_for_tick()
            
            robot_action = RobotAction(joint_cmds=joint_cmds)
            robot_action = self.robot.apply_action(robot_action)
            self._control_log.append(robot_action)

        logger.info("Quitting control loop.")
        # Set stop
        self.robot.pause()

    def read_loop(self) -> None:
        # start_time = time.time()
        read_rate_limiter = RateLimiterSync(self.read_freq)
        while self.robot.is_active():
            try:
                read_rate_limiter.wait_for_tick()
                
                state = self.robot.get_state()

            except Exception as e:
                break
            self._read_log.append(state)
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
    def __init__(self, robot, joint_cmds_traj: Union[list[list[float]], list[list[list[float]]]], control_freq: int = 100, read_freq: int = 100):
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
        control_rate_limiter = RateLimiter(self.control_freq)
        for joint_cmds in self.joint_cmds_traj:
            await control_rate_limiter.wait_for_tick()

            robot_action = RobotAction(joint_cmds=joint_cmds)
            robot_action = await AsyncUtil.run_blocking_as_async(self.robot.apply_action, robot_action)
            self._control_log.append(robot_action)

            await control_rate_limiter.unset_busy()
        logger.info("Quitting control loop.")
        # Set stop
        await AsyncUtil.run_blocking_as_async(self.robot.pause)

    async def read_loop(self):
        read_rate_limiter = RateLimiter(self.read_freq)
        while self.robot.is_active():
            try:
                await read_rate_limiter.wait_for_tick()

                state = await AsyncUtil.run_blocking_as_async(self.robot.get_state)

                await read_rate_limiter.unset_busy()
            except Exception as e:
                break
            self._read_log.append(state)
            await TimeUtil.sleep_by_ms(1000/self.read_freq)
        logger.info("Quitting read loop.")