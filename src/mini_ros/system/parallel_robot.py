"""
Parallel robot system.
Support multiple-thread & asyncio.
"""
from math import log
from copy import deepcopy
import matplotlib.pyplot as plt
import threading
import time
import asyncio
import numpy as np
from typing import Union, Dict, Any, List
from loguru import logger
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.common.device import Robot, CameraData
from mini_ros.wrapper.multi_robot import MultiRobotCamera
from mini_ros.common.state import RobotAction, RobotState
from mini_ros.utils.rate_limiter import RateLimiterAsync, RateLimiterSync


class ParallelRobotMultiThread:
    """
    Parallel robot system.
    Support multiple-thread & asyncio.
    """
    def __init__(self, multi_robot: MultiRobotCamera, control_freqs: Dict[str, int], read_freqs: Dict[str, int]):
        self.multi_robot: MultiRobotCamera = multi_robot
        self.control_freqs = control_freqs
        self.read_freqs = read_freqs
        self.stop_event = threading.Event()
        self.stop_event.clear()
        # Control buffer
        # This control buffer can update in multi-thread's way
        self.control_mutex: Dict[str, threading.Lock] = {robot_name: threading.Lock() for robot_name in self.multi_robot.robots.keys()}
        self.control_buffer: Dict[str, RobotAction] = {robot_name: None for robot_name in self.multi_robot.robots.keys()}

    def start(self):
        self.control_threads = []
        self.read_threads = []
        for device_name in self.multi_robot.device_keys:
            if isinstance(self.multi_robot.device(device_name), Robot):
                self.control_threads.append(threading.Thread(target=self.control_loop, args=(device_name,)))
                self.control_threads[-1].start()
            self.read_threads.append(threading.Thread(target=self.read_loop, args=(device_name,)))
            self.read_threads[-1].start()
        
        self.update_control_threads = self.get_update_control_threads()
        for update_control_thread in self.update_control_threads:
            update_control_thread.start()

    def join(self):
        for control_thread, read_thread, update_control_thread in zip(self.control_threads, self.read_threads, self.update_control_threads):
            control_thread.join()
            read_thread.join()
            update_control_thread.join()

    def get_update_control_threads(self) -> List[threading.Thread]:
        """
        Update the control loop.
        """
        raise NotImplementedError("Not implemented yet!")

    def control_loop(self, robot_name: str) -> None:
        """
        Loop control the data.
        """
        control_rate_limiter = RateLimiterSync(self.control_freqs[robot_name])
        logger.info(f"Starting control thread for robot {robot_name}. Active: {self.multi_robot.is_active()}")
        last_robot_action: RobotAction = None
        while True:
            if self.stop_event.is_set() or not self.multi_robot.is_alive():
                break

            try:
                control_rate_limiter.wait_for_tick()
                with self.control_mutex[robot_name]:
                    if self.control_buffer[robot_name] is not None:
                        robot_action = self.control_buffer[robot_name]
                        self.control_buffer[robot_name] = None
                        last_robot_action = deepcopy(robot_action)
                    else:
                        if last_robot_action is not None:
                            robot_action = deepcopy(last_robot_action)
                        else:
                            raise ValueError(f"No robot action to apply for robot {robot_name}!")

                timed_robot_action = self.multi_robot.apply_action_to(robot_name, robot_action, is_record=True)
            except Exception as e:
                logger.error(f"Error in control loop for robot {robot_name}: {type(e).__name__}: {e}")
                break

    def read_loop(self, device_name: str, episode_length: int = 4) -> None:
        """
        Reading loop. Switch to next episode after episode_length seconds.
        """
        read_rate_limiter = RateLimiterSync(self.read_freqs[device_name])
        logger.info(f"Starting read loop for robot {device_name}. Active: {self.multi_robot.is_active()}")
        episode_idx = 0
        while True:
            if self.stop_event.is_set() or not self.multi_robot.is_alive():
                break

            try:
                read_rate_limiter.reset()
                self.multi_robot.start_record_at(device_name=device_name, episode_name=f"e_{episode_idx:06d}")
                episode_idx += 1
            except Exception as e:
                logger.error(f"Error in start record for {device_name}: {e}")
                break
            start_time = TimeUtil.now()
            while (TimeUtil.now() - start_time).total_seconds() < episode_length:
                try:
                    read_rate_limiter.wait_for_tick()
                    state = self.multi_robot.get_state_from(device_name, is_record=True)
                except Exception as e:
                    logger.error(f"Error in read loop for {device_name}: {type(e).__name__}: {e}")
                    break
            self.multi_robot.stop_record_at(device_name=device_name)
            logger.info(f"Start waiting for next episode {device_name}")
            time.sleep(2)
            logger.info(f"Switch to next episode at {episode_idx:06d}")
        logger.info(f"Quitting read loop for robot {device_name}.")