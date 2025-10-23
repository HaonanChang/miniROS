from typing import Any, List, Dict
from loguru import logger
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.common.device import Robot, RobotAction, RobotState, Camera


class MultiRobot(Robot):
    """
    Wrapper for multiple robots.
    You can load multiple robots on one union interface.
    """
    name: str = "multi_robot"

    def __init__(self, robots: Dict[str, Robot]):
        self.robots = robots

    def initialize(self):
        """
        Sequentially initialize all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].initialize()

    def start(self):
        """
        Sequentially start all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].start()

    def stop(self):
        """
        Sequentially stop all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].stop()

    def pause(self):
        """
        Sequentially pause all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].pause()

    def reboot(self):
        """
        Sequentially reboot all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].reboot()

    def get_state(self, timeout: float = 1.0) -> RobotState:
        raise RuntimeError("get_state is not supported for multi-robot")

    def apply_action(self, action: RobotAction):
        raise RuntimeError("apply_action is not supported for multi-robot")

    def apply_action_to(self, robot_name: str, action: RobotAction):
        """
        Apply action to a specific robot.
        """
        if robot_name not in self.robots.keys():
            logger.error(f"Robot {robot_name} not found in multi-robot")
            raise ValueError(f"Robot {robot_name} not found in multi-robot")
        return self.robots[robot_name].apply_action(action)

    def get_state_from(self, robot_name: str, timeout: float = 1.0) -> RobotState:
        """
        Get state from a specific robot.
        """
        if robot_name not in self.robots.keys():
            logger.error(f"Robot {robot_name} not found in multi-robot")
            raise ValueError(f"Robot {robot_name} not found in multi-robot")
        return self.robots[robot_name].get_state(timeout)

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


class MultiRobotCamera(Robot):
    """
    Wrapper for multiple robots and multiple cameras.
    You can load multiple robots on one union interface.
    """
    name: str = "multi_robot"

    def __init__(self, robots: Dict[str, Robot], cameras: Dict[str, Camera]):
        self.robots = robots
        self.cameras = cameras

    def initialize(self):
        """
        Sequentially initialize all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].initialize()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].initialize()

    def start(self):
        """
        Sequentially start all robots.
        """
        for robot_name in self.robots.keys():
            self.robots[robot_name].start()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].start()

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
            
    def get_state(self, timeout: float = 1.0) -> RobotState:
        raise RuntimeError("get_state is not supported for multi-robot")

    def apply_action(self, action: RobotAction):
        raise RuntimeError("apply_action is not supported for multi-robot")

    def apply_action_to(self, robot_name: str, action: RobotAction):
        """
        Apply action to a specific robot.
        """
        if robot_name not in self.robots.keys():
            logger.error(f"Robot {robot_name} not found in multi-robot")
            raise ValueError(f"Robot {robot_name} not found in multi-robot")
        return self.robots[robot_name].apply_action(action)

    def get_state_from(self, robot_name: str, timeout: float = 1.0) -> RobotState:
        """
        Get state from a specific robot.
        """
        if robot_name not in self.robots.keys():
            logger.error(f"Robot {robot_name} not found in multi-robot")
            raise ValueError(f"Robot {robot_name} not found in multi-robot")
        return self.robots[robot_name].get_state(timeout)

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