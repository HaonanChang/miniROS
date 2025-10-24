from typing import Any, List, Dict
from loguru import logger
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.devices.io.recorder import EpisodeRecorder
from mini_ros.common.device import Robot, RobotAction, RobotState, Camera, Recorder, Device


class MultiRobotSystem(Robot):
    """
    Wrapper for multiple robots and multiple cameras.
    You can load multiple robots on one union interface.
    """
    name: str = "multi_robot"

    def __init__(self, devices: List[Robot | Camera | Recorder | Device] = []):
        """
        """
        self.devices = {}
        self.robots = {}
        self.cameras = {}
        self.recorders = {}
        for device in devices:
            device_name = device.name
            if isinstance(device, Robot):
                self.robots[device_name] = device
            elif isinstance(device, Camera):
                self.cameras[device_name] = device
            elif isinstance(device, Recorder):
                self.recorders[device_name] = device
            elif isinstance(device, Device):
                # Other devices
                self.devices[device_name] = device
            else:
                raise ValueError(f"Device {device_name} is not a valid device")

        # Bind recorders to robots
        for robot_name in self.robots.keys():
            if robot_name in self.recorders.keys():
                self.robots[robot_name].bind_recorder(self.recorders[robot_name])
            else:
                logger.warning(f"Recorder for robot {robot_name} not found, will not record data")

    def initialize(self):
        """
        Sequentially initialize all robots.
        """
        for device_name in self.device_keys:
            self.device(device_name).initialize()

    def start(self):
        """
        NOTE: Start is a blocking call.
        Sequentially start all robots.
        """
        for device_name in self.device_keys:
            self.device(device_name).start()

    def stop(self):
        """
        Sequentially stop all robots.
        """
        for device_name in self.device_keys:
            self.device(device_name).stop()
            
    def pause(self):
        """
        Sequentially pause all robots.
        """
        for device_name in self.device_keys:
            self.device(device_name).pause()
    
    def start_record_at(self, device_name: str, episode_name: str):
        """
        Sequentially start recording a specific device.
        """
        if device_name not in self.device_keys:
            logger.error(f"Device {device_name} not found in multi-robot")
            raise ValueError(f"Device {device_name} not found in multi-robot")
        self.device(device_name).start_record(episode_name)

    def stop_record_at(self, device_name: str):
        """
        Sequentially stop recording a specific device.
        """
        if device_name not in self.device_keys:
            logger.error(f"Device {device_name} not found in multi-robot")
            raise ValueError(f"Device {device_name} not found in multi-robot")
        self.device(device_name).stop_record()

    def reboot(self):
        """
        Sequentially reboot all robots.
        """
        for device_name in self.device_keys:
            self.device(device_name).reboot()
            
    def get_state(self, timeout: float = 1.0, is_record: bool = False) -> RobotState:
        raise RuntimeError("get_state is not supported for multi-robot")

    def apply_action(self, action: RobotAction, is_record: bool = False):
        raise RuntimeError("apply_action is not supported for multi-robot")

    def apply_action_to(self, robot_name: str, action: RobotAction, is_record: bool = False):
        """
        Apply action to a specific robot.
        """
        if robot_name not in self.robots.keys():
            logger.error(f"Robot {robot_name} not found in multi-robot")
            raise ValueError(f"Robot {robot_name} not found in multi-robot")
        return self.robots[robot_name].apply_action(action, is_record)

    def get_state_from(self, device_name: str, timeout: float = 1.0, is_record: bool = False) -> RobotState:
        """
        Get state from a specific device.
        """
        if device_name not in self.device_keys:
            logger.error(f"Device {device_name} not found in multi-robot")
            raise ValueError(f"Device {device_name} not found in multi-robot")
        return self.device(device_name).get_state(timeout=timeout, is_record=is_record)

    def is_active(self) -> bool:
        action_flag = True
        for robot_name in self.robots.keys():
            if not self.robots[robot_name].is_active():
                action_flag = False
                logger.warning(f"Robot {robot_name} is not active")
                break
        return action_flag
    
    def is_alive(self) -> bool:
        alive_flag = True
        for robot_name in self.robots.keys():
            if not self.robots[robot_name].is_alive():
                alive_flag = False
                logger.warning(f"Robot {robot_name} is not alive")
                break
        return alive_flag
    
    @property
    def num_dof(self) -> int:
        num_dof = 0
        for robot_name in self.robots.keys():
            num_dof += self.robots[robot_name].num_dof
        return num_dof
    
    @property
    def max_control_freq(self) -> int:
        raise RuntimeError("max_control_freq is not supported for multi-robot")

    @property
    def max_read_freq(self) -> int:
        raise RuntimeError("max_read_freq is not supported for multi-robot")

    @property
    def max_control_freq_at(self, robot_name: str) -> int:
        return self.robots[robot_name].max_control_freq

    @property
    def max_read_freq_at(self, robot_name: str) -> int:
        return self.robots[robot_name].max_read_freq

    # Idle methods
    def _initialize_robot(self):
        raise RuntimeError("initialize_robot is not supported for multi-robot")

    def _start_robot(self):
        raise RuntimeError("start_robot is not supported for multi-robot")

    def _pause_robot(self):
        raise RuntimeError("pause_robot is not supported for multi-robot")

    def _stop_robot(self):
        raise RuntimeError("stop_robot is not supported for multi-robot")

    @property
    def device_keys(self) -> List[str]:
        return list(self.robots.keys()) + list(self.cameras.keys())

    def device(self, device_name: str) -> Robot | Camera:
        if device_name in self.robots.keys():
            return self.robots[device_name]
        elif device_name in self.cameras.keys():
            return self.cameras[device_name]
        else:
            logger.error(f"Device {device_name} not found in multi-robot")
            return None