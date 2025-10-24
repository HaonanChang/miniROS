from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import numpy as np
from mini_ros.common.state import RobotState, RobotAction, CameraData
import threading
from loguru import logger


############################## Device Base Class ##############################
class Device(ABC):
    """
    Device base class.
    Base device is supposed to be read-only.
    """
    
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def initialize(self):
        raise NotImplementedError("initialize method is not implemented")

    @abstractmethod
    def start(self):
        raise NotImplementedError("start method is not implemented")

    @abstractmethod
    def stop(self):
        raise NotImplementedError("stop method is not implemented")
        
    @abstractmethod
    def pause(self):
        raise NotImplementedError("pause method is not implemented")

    @abstractmethod
    def start_record(self, episode_name: str):
        raise NotImplementedError("start_record method is not implemented")

    @abstractmethod
    def stop_record(self):
        raise NotImplementedError("stop_record method is not implemented")


class Recorder(Device):
    def __init__(self, config: Any):
        pass

    @abstractmethod
    def save(self):
        """
        NOTE: Save is a blocking call.
        """
        pass

    @abstractmethod
    def apply_action(self, data: Any, key: str):
        """
        Put the data into the data buffer.
        NOTE: Put must be a non-blocking call. 
        Because it will be called in other time-sensitive threads.
        """
        pass


class Robot(Device):
    """
    Base class for all robots.
    """
    name: str
    is_full_duplex: bool = True

    def __init__(self):
        self.recorder: Recorder = None
        # Active: Can be controlled
        self._active_event = threading.Event()
        # Connect: Connected to the robot
        self._connect_event = threading.Event()

    ###################### Interface Methods ######################
    def bind_recorder(self, recorder: Recorder):
        self.recorder = recorder

    def initialize(self):
        """
        Initialize the robot.
        """
        flag = self._initialize_robot()
        if not flag:
            logger.error(f"Failed to initialize robot: {self.name}")
            return False
        self._connect_event.set()
        return True

    def start(self):
        """
        NOTE: Start is a blocking call.
        """
        if not self.is_alive():
            # Can't be double started
            logger.warning(f"{self.name} is not connected, can't be started")
            return
        self._start_robot()

        if self.recorder is not None:
            self.recorder.start()
        # Set active event
        self._active_event.set()

    def start_record(self, episode_name: str):
        """
        NOTE: Start recording is a blocking call.
        Start recording the robot.
        """
        if self.recorder is not None:
            self.recorder.start_record(episode_name)
        else:
            logger.warning(f"Recorder is not bound to {self.name}, can't start recording")
    
    def stop_record(self):
        """
        NOTE: Stop recording is a blocking call.
        Stop recording the robot.
        """
        if self.recorder is not None:
            logger.info(f"Stopping recording: {self.name}")
            self.recorder.stop_record()
        else:
            logger.warning(f"Recorder is not bound to {self.name}, can't stop recording")
            
    def stop(self):
        """
        NOTE: Stop is a blocking call.
        """
        if not self.is_alive():
            # Can't be double stopped
            logger.warning(f"{self.name} is not connected, can't be stopped")
            return
        self._active_event.clear()
        self._connect_event.clear()
        self._stop_robot()
        if self.recorder is not None:
            self.recorder.stop()

    def pause(self):
        """
        NOTE: Pause is a blocking call.
        """
        if not self.is_active():
            # Can't be double paused
            logger.warning(f"{self.name} is not active, can't be paused")
            return
        self._active_event.clear()
        logger.info(f"Pausing {self.name}: Active: {self.is_active()}, Alive: {self.is_alive()}")
        self._pause_robot()

    def is_active(self) -> bool:
        return self._active_event.is_set()
    
    def is_alive(self) -> bool:
        return self._connect_event.is_set()

    ###################### Required Methods ######################
    @abstractmethod
    def get_state(self, timeout: float = 1.0, is_record: bool = False) -> RobotState:
        """
        Get the state of the robot.
        Args:
            timeout: Timeout in seconds
            is_record: Whether to record the state if binded with a recorder
        Returns:
            RobotState: The state of the robot
        """
        raise NotImplementedError("get_state method is not implemented")

    @abstractmethod
    def apply_action(self, action: RobotAction, is_record: bool = False):
        raise NotImplementedError("apply_action method is not implemented")

    @abstractmethod
    def _initialize_robot(self):
        raise NotImplementedError("initialize_robot method is not implemented")

    @abstractmethod
    def _start_robot(self):
        raise NotImplementedError("start_robot method is not implemented")

    @abstractmethod
    def _pause_robot(self):
        raise NotImplementedError("pause_robot method is not implemented")

    @abstractmethod
    def _stop_robot(self):
        raise NotImplementedError("stop_robot method is not implemented")

    @abstractmethod
    def reboot(self):
        raise NotImplementedError("reboot method is not implemented")

    @property
    def num_dof(self) -> int:
        raise NotImplementedError("num_dof method is not implemented")

    @property
    def max_control_freq(self) -> int:
        raise NotImplementedError("max_control_freq method is not implemented")

    @property
    def max_read_freq(self) -> int:
        raise NotImplementedError("max_read_freq method is not implemented")

    ###################### Optional Methods ######################
    def pause(self):
        """
        [Optional]
        Pause the robot. If not implemented, it will call stop() by default.
        """
        self.stop()
        
    def set_safe_speed(self):
        """
        [Optional]
        Set robot to safe speed.
        """
        pass

    def set_normal_speed(self):
        """
        [Optional]
        Set robot to normal speed.
        """
        pass

    def switch_mode(self, mode: str):
        """
        [Optional]
        Switch robot between different modes.
        """
        pass

    def is_state_valid(self, state: RobotState) -> bool:
        """
        Check if the robot state is valid given the robot type.
        """
        return True

    def is_action_valid(self, action: RobotAction) -> bool:
        """
        Check if the robot action is valid given the robot type.
        """
        return True


class Camera(Device):
    """
    Base class for all cameras.
    Camera is one type of Devices. 
    The major difference is that Camera is IO-heavy, so we can choose to save the data locally on fly.
    """
    name: str

    @abstractmethod
    def initialize(self):
        raise NotImplementedError("initialize method is not implemented")

    @abstractmethod
    def bind_recorder(self, recorder: Recorder):
        raise NotImplementedError("bind_recorder method is not implemented")

    @abstractmethod
    def start(self):
        raise NotImplementedError("start method is not implemented")

    @abstractmethod
    def stop(self):
        raise NotImplementedError("stop method is not implemented")

    @abstractmethod
    def start_record(self, episode_name: str):
        raise NotImplementedError("start_record method is not implemented")

    @abstractmethod
    def stop_record(self):
        raise NotImplementedError("stop_record method is not implemented")

    @abstractmethod
    def pause(self):
        raise NotImplementedError("pause method is not implemented")

    @abstractmethod
    def get_frame(self) -> Any:
        """
        Polling from device
        """
        raise NotImplementedError("get_frame method is not implemented")

    @abstractmethod
    def process_frame(self, frame: Any) -> CameraData:
        """
        Process the frame from device and return the camera data
        """
        raise NotImplementedError("process_frame method is not implemented")

    @abstractmethod
    def save_data(self, path: Optional[str] = None):
        """
        Camera is IO-heavy, so we can choose to save the data locally on fly.
        """
        raise NotImplementedError("save_data method is not implemented")

    @abstractmethod
    def is_active(self) -> bool:
        """
        Check if the camera is active.
        """
        raise NotImplementedError("is_active method is not implemented")

    @abstractmethod
    def is_alive(self) -> bool:
        """
        Check if the camera is alive.
        """
        raise NotImplementedError("is_alive method is not implemented")

    ###################### Optional Methods ######################
    def reboot(self):
        """
        [Optional]
        Reboot the camera.
        """
        pass

################################# Motor Device #################################
@dataclass
class MotorConfig:
    """
    Motor config. for joint mapping.

    if is_rot:
        raw_angle = warp_to_pi(raw_angle)
    angle = sign * (raw_angle - zero_point) * scale + ref_angle
    angle = clip(angle, lower_limit, upper_limit)
    """
    port: str
    id: int
    joint_name: str
    motor_type: str = "motor"
    sign: int = 1
    zero_point: float = 0.0
    scale: float = 1.0
    ref_point: float = 0.0
    upper_limit: float = np.pi * 2
    lower_limit: float = -np.pi * 2
    is_rot: bool = True


def motor_config_from_json(config_json: Dict[str, Any], include_names: List[str] = []) -> List[MotorConfig]:
    """
    Convert a JSON config to a list of MotorConfig.
    """
    joint_config = []
    for joint_name, joint_info in config_json.items():
        if include_names and joint_name not in include_names:
            continue
        joint_config.append(MotorConfig(joint_name=joint_name, **joint_info))
    return joint_config


class MotorReader(Device):
    """
    Motor driver interface (Blocking version).
    Implement this method.
    """

    def __init__(self, motor_configs: List[MotorConfig]):
        pass

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def get_state(self):
        pass

    @classmethod
    def calibrate_read(cls, raw_read: float, joint_config: MotorConfig) -> float:
        """
        Calibrate the raw read to the joint config.
        Args:
            raw_read: raw read from the motor (Already converted to degree if is_rot)
            joint_config: joint config
        Returns:
            corrected_read: corrected read
        """
        sign = joint_config.sign
        zero_point = joint_config.zero_point
        scale = joint_config.scale
        ref_point = joint_config.ref_point
        upper_limit = joint_config.upper_limit
        lower_limit = joint_config.lower_limit
        is_rot = joint_config.is_rot
        
        if is_rot:
            raw_read = np.mod(raw_read + 180, 360) - 180
            raw_read = np.deg2rad(raw_read)
        
        # Apply zero point correction
        corrected_read = raw_read - zero_point
        
        # Apply sign and gain corrections
        corrected_read = sign * corrected_read * scale + ref_point
        
        if is_rot:
            # Round
            corrected_read = np.mod(corrected_read + np.pi, np.pi * 2) - np.pi

        # Clip by limit
        corrected_read = np.clip(corrected_read, lower_limit, upper_limit)
        
        return corrected_read


################################# Tracker Device #################################
class Tracker(Device):
    """
    Base class for all 6D pose trackers.
    """
    name: str

    @abstractmethod
    def reanchor(self):
        """
        Reanchor the tracker to the current pose.
        """
        pass