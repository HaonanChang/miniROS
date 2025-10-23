from typing import Any, List, Dict
from loguru import logger
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.devices.io.recorder import EpisodeRecorder
from mini_ros.common.device import Robot, RobotAction, RobotState, Camera, Recorder


class MultiRobotCamera(Robot):
    """
    Wrapper for multiple robots and multiple cameras.
    You can load multiple robots on one union interface.
    """
    name: str = "multi_robot"

    def __init__(self, robots: Dict[str, Robot], cameras: Dict[str, Camera], recorders: Dict[str, Recorder]):
        """
        """
        self.robots = robots
        self.cameras = cameras
        self.recorders = recorders
        # Bind recorders to robots and cameras
        for robot_name in self.robots.keys():
            if robot_name in self.recorders.keys():
                self.robots[robot_name].bind_recorder(self.recorders[robot_name])
            else:
                logger.warning(f"Recorder for robot {robot_name} not found, will not record data")
        for camera_name in self.cameras.keys():
            if camera_name in self.recorders.keys():
                self.cameras[camera_name].bind_recorder(self.recorders[camera_name])
            else:
                logger.warning(f"Recorder for camera {camera_name} not found, will not record data")

    def initialize(self):
        """
        Sequentially initialize all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].initialize()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].initialize()

    def start(self, episode_name: str):
        """
        NOTE: Start is a blocking call.
        Sequentially start all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].start(episode_name)
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].start(episode_name)

    def stop(self):
        """
        Sequentially stop all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].stop()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].stop()
            
    def pause(self):
        """
        Sequentially pause all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].pause()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].pause()

    def reboot(self):
        """
        Sequentially reboot all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].reboot()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].reboot()

    def save(self, device_name: str):
        """
        Sequentially save all recorders.
        """
        if device_name not in self.recorders.keys():
            logger.error(f"Recorder for {device_name} not found in multi-robot")
        self.recorders[device_name].save()
            
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

    def get_state_from(self, robot_name: str, timeout: float = 1.0, is_record: bool = False) -> RobotState:
        """
        Get state from a specific robot.
        """
        if robot_name not in self.robots.keys():
            logger.error(f"Robot {robot_name} not found in multi-robot")
            raise ValueError(f"Robot {robot_name} not found in multi-robot")
        return self.robots[robot_name].get_state(timeout, is_record)

    def is_active(self) -> bool:
        action_flag = True
        for robot_name in self.robots.keys():
            if not self.robots[robot_name].is_active():
                action_flag = False
                logger.warning(f"Robot {robot_name} is not active")
                break
        return action_flag
    
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