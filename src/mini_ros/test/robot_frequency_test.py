"""
Ultilify functions for testing robot frequency.
"""
from math import log
import matplotlib.pyplot as plt
import threading
import time
import asyncio
import numpy as np
from typing import Union, Dict, Any
from loguru import logger
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.common.device import Robot
from mini_ros.wrapper.multi_robot import MultiRobot
from mini_ros.common.state import RobotAction, RobotState
from mini_ros.utils.rate_limiter import RateLimiterAsync, RateLimiterSync


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


def draw_compare_fig(control_log: list[RobotAction], read_log: list[RobotState], compare_type: str = "joint_cmds", title="", y_margin=0.0) -> None:
    """
    Generate the comparison figure of the control and read log.
    """
    if compare_type == "joint_cmds":
        control_timestamp = [action.timestamp for action in control_log]
        control_joint_cmds = [action.joint_cmds for action in control_log]
        control_joint_cmds = np.array(control_joint_cmds)
        # control_joint_cmds -= control_joint_cmds[0]

        read_timestamp = [state.timestamp for state in read_log]
        read_joint_positions = [state.joint_positions for state in read_log]
        read_joint_positions = np.array(read_joint_positions)
        # read_joint_positions -= read_joint_positions[0]

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


################################### Mutli robot test ###################################
class FreqTestMultiRobot:
    """
    Test the robot read & write method into multi-thread.
    """
    def __init__(self, multi_robot: MultiRobot, joint_cmds_traj: Dict[str, list[list[float]]], control_freqs: Dict[str, int], read_freqs: Dict[str, int]):
        self.multi_robot: MultiRobot = multi_robot
        self.joint_cmds_traj = joint_cmds_traj
        self.control_freqs = control_freqs
        self.read_freqs = read_freqs

        # Log
        self._control_log: Dict[str, list[RobotAction]] = {robot_name: [] for robot_name in self.multi_robot.robots.keys()}
        self._read_log: Dict[str, list[RobotState]] = {robot_name: [] for robot_name in self.multi_robot.robots.keys()}

    def start(self):
        pass

    def start(self):
        pass

    def join(self):
        pass
    def generate_compare_fig(self, compare_type: str = "joint_cmds", title="", y_margin=0.0) -> None:
        for robot_name in self.multi_robot.robots.keys():
            draw_compare_fig(control_log=self._control_log[robot_name], read_log=self._read_log[robot_name], compare_type=compare_type, title=f"{title}_{robot_name}", y_margin=y_margin)


class MultiThreadFreqTestMultiRobot(FreqTestMultiRobot):
    """
    Test the robot read & write method into multi-thread.
    """
    def __init__(self, multi_robot: MultiRobot, joint_cmds_traj: Dict[str, list[list[float]]], control_freqs: Dict[str, int], read_freqs: Dict[str, int]):
        self.multi_robot: MultiRobot = multi_robot
        self.joint_cmds_traj = joint_cmds_traj
        self.control_freqs = control_freqs
        self.read_freqs = read_freqs

        # Log
        self._control_log: Dict[str, list[RobotAction]] = {robot_name: [] for robot_name in self.multi_robot.robots.keys()}
        self._read_log: Dict[str, list[RobotState]] = {robot_name: [] for robot_name in self.multi_robot.robots.keys()}

    def start(self):
        self.control_threads = []
        self.read_threads = []
        print(self.multi_robot.robots.keys())
        for robot_name in self.multi_robot.robots.keys():
            logger.info(f"Starting control thread for robot {robot_name}. Active: {self.multi_robot.is_active()}")
            self.control_threads.append(threading.Thread(target=self.control_loop, args=(robot_name,)))
            self.read_threads.append(threading.Thread(target=self.read_loop, args=(robot_name,)))
            self.control_threads[-1].start()
            self.read_threads[-1].start()

    def join(self):
        for control_thread, read_thread in zip(self.control_threads, self.read_threads):
            control_thread.join()
            read_thread.join()

    def control_loop(self, robot_name: str) -> None:
        control_rate_limiter = RateLimiterSync(self.control_freqs[robot_name])
        for joint_cmds in self.joint_cmds_traj[robot_name]:
            control_rate_limiter.wait_for_tick()

            robot_action = RobotAction(joint_cmds=joint_cmds)
            robot_action = self.multi_robot.apply_action_to(robot_name, robot_action)
            self._control_log[robot_name].append(robot_action)
        # Set stop
        self.multi_robot.pause()
        logger.info(f"Quitting control loop for robot {robot_name}.")

    def read_loop(self, robot_name: str) -> None:
        read_rate_limiter = RateLimiterSync(self.read_freqs[robot_name])
        logger.info(f"Starting read loop for robot {robot_name}. Active: {self.multi_robot.is_active()}")
        while self.multi_robot.is_active():
            try:
                read_rate_limiter.wait_for_tick()
                state = self.multi_robot.get_state_from(robot_name)
                self._read_log[robot_name].append(state)
            except Exception as e:
                logger.error(f"Error in read loop for robot {robot_name}: {e}")
                break
        logger.info(f"Quitting read loop for robot {robot_name}.")


class AsyncFreqTestMultiRobot(FreqTestMultiRobot):
    """
    Test the robot read & write method into multi-thread.
    """
    def __init__(self, multi_robot: MultiRobot, joint_cmds_traj: Dict[str, list[list[float]]], control_freqs: Dict[str, int], read_freqs: Dict[str, int]):
        self.multi_robot: MultiRobot = multi_robot
        self.joint_cmds_traj = joint_cmds_traj
        self.control_freqs = control_freqs
        self.read_freqs = read_freqs

        self.stop_mutex = asyncio.Lock()

        # Log
        self._control_log: Dict[str, list[RobotAction]] = {robot_name: [] for robot_name in self.multi_robot.robots.keys()}
        self._read_log: Dict[str, list[RobotState]] = {robot_name: [] for robot_name in self.multi_robot.robots.keys()}

    def start(self):
        self.control_tasks = [asyncio.create_task(self.control_loop(robot_name)) for robot_name in self.multi_robot.robots.keys()]
        self.read_tasks = [asyncio.create_task(self.read_loop(robot_name)) for robot_name in self.multi_robot.robots.keys()]
        # AsyncUtil.run_async_as_blocking(asyncio.gather(*self.control_tasks, *self.read_tasks))
        
    async def join(self):
        try:
            await asyncio.gather(*self.control_tasks, *self.read_tasks)
        except asyncio.CancelledError as e:
            logger.info(f"Join cancelled: {e}")
        except Exception as e:
            logger.error(f"Error in join: {e}")
        finally:
            self.multi_robot.pause()

    async def control_loop(self, robot_name: str) -> None:
        control_rate_limiter = RateLimiterAsync(self.control_freqs[robot_name])
        for joint_cmds in self.joint_cmds_traj[robot_name]:
            try:
                await control_rate_limiter.wait_for_tick()

                robot_action = RobotAction(joint_cmds=joint_cmds)
                robot_action = await AsyncUtil.run_blocking_as_async(self.multi_robot.apply_action_to, robot_name, robot_action)
                self._control_log[robot_name].append(robot_action)
            except asyncio.CancelledError:
                logger.info(f"Control loop for robot {robot_name} cancelled.")
                break
            except Exception as e:
                logger.error(f"Error in control loop for robot {robot_name}: {e}")
                break
        # Set stop
        logger.info(f"Quitting control loop for robot {robot_name}.")
        await self.stop()

    async def read_loop(self, robot_name: str) -> None:
        read_rate_limiter = RateLimiterAsync(self.read_freqs[robot_name])
        logger.info(f"Starting read loop for robot {robot_name}. Active: {self.multi_robot.is_active()}")
        while self.multi_robot.is_active():
            try:
                await read_rate_limiter.wait_for_tick()
                state = await AsyncUtil.run_blocking_as_async(self.multi_robot.get_state_from, robot_name)
                self._read_log[robot_name].append(state)
            except Exception as e:
                logger.error(f"Error in read loop for robot {robot_name}: {e}")
                break
            except asyncio.CancelledError:
                logger.info(f"Read loop for robot {robot_name} cancelled.")
                break
        logger.info(f"Quitting read loop for robot {robot_name}.")
    
    async def stop(self):
        # Only one instance can stop the test at a time.
        async with self.stop_mutex:
            logger.info("Stopping multi-robot test.")
            # Cancel all tasks
            for control_task, read_task in zip(self.control_tasks, self.read_tasks):
                control_task.cancel()
                read_task.cancel()
