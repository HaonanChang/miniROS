from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import numpy as np
from mini_ros.common.state import RobotState, RobotAction, CameraData


############################## Device Base Class ##############################
class Device(ABC):
    """
    Device base class
    """
    
    def __init__(self, name: str):
        self.name = name


class Reader(ABC):
    """
    Base class for all readers.
    """
    name: str

    @abstractmethod
    def initialize(self, driver_config: Any):
        pass

    @abstractmethod
    def get_state(self) -> Any:
        pass

    @abstractmethod
    def stop(self):
        pass


class Robot(ABC):
    """
    Base class for all robots.
    """
    name: str
    is_full_duplex: bool = True

    ###################### Required Methods ######################
    @abstractmethod
    def initialize(self, driver_config: Any):
        pass

    @abstractmethod
    def get_state(self, timeout: float = 1.0) -> RobotState:
        # State can be hold in a pulling thread
        pass

    @abstractmethod
    def apply_action(self, action: RobotAction):
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def reboot(self):
        pass

    def is_active(self) -> bool:
        """
        Check if the robot is active.
        """
        raise NotImplementedError("is_active method is not implemented")

    def is_alive(self) -> bool:
        """
        Check if the robot is alive.
        """
        raise NotImplementedError("is_alive method is not implemented")

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


class Streamer(ABC):
    """
    Base class for all stream drivers. Stream driver has a long-running process and perform processing in callback.
    """
    name: str

    @abstractmethod
    def initialize(self, driver_config: Any):
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def on_new_frame(self):
        pass

    @abstractmethod
    def on_stream_stopped(self):
        pass


class Camera:
    """
    Base class for all cameras.
    """
    name: str

    @abstractmethod
    def initialize(self, driver_config: Any):
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def pause(self):
        pass

    @abstractmethod
    def get_frame(self) -> Any:
        """
        Polling from device
        """
        pass

    @abstractmethod
    def process_frame(self, frame: Any) -> CameraData:
        """
        Process the frame from device and return the camera data
        """
        pass

    @abstractmethod
    def save_data(self, path: Optional[str] = None):
        """
        Camera is IO-heavy, so we can choose to save the data locally on fly.
        """
        pass

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


class MotorReader(Reader):
    """
    Motor driver interface (Blocking version).
    Implement this method.
    """

    def __init__(self):
        pass

    def initialize(self, joint_config: List[MotorConfig]):
        self.joint_config = joint_config

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
class Tracker(Reader):
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