from abc import ABC, abstractmethod
from typing import Dict, Any, List
from mini_ros.utils.async_util import AsyncUtil
from dataclasses import dataclass
import numpy as np


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
    is_rot: bool = False


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



class MotorDriver(ABC):
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
        
        # Clip by limit
        corrected_read = np.clip(corrected_read, lower_limit, upper_limit)
        
        return corrected_read


class MotorDriverAsyncWrapper:
    """
    Motor driver interface (Async version).
    You don't need to implement, just pass the motor driver to this class.
    """

    def __init__(self, motor_driver: MotorDriver):
        self.motor_driver = motor_driver

    async def initialize(self, joint_config: Dict[str, Dict[str, Any]]):
        await AsyncUtil.run_blocking_as_async(self.motor_driver.initialize, joint_config)

    async def stop(self):
        await AsyncUtil.run_blocking_as_async(self.motor_driver.stop)

    async def get_state(self):
        return await AsyncUtil.run_blocking_as_async(self.motor_driver.get_state)