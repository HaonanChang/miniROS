"""
General mocked driver, you can define the input you want.
"""

from typing import Optional, Sequence, List
from dataclasses import dataclass
import numpy as np
from loguru import logger
from mini_ros.common.device import MotorReader, MotorConfig, motor_config_from_json


@dataclass
class MockMotorConfig:
    id: int


class MockMotorReader(MotorReader):
    """
    A mock motor reader, you can define the angle you want
    """

    def __init__(
        self,
        motor_configs: List[MockMotorConfig],
    ):
        self.motor_configs = motor_configs

    def name(self) -> str:
        return "mock_motor"

    def initialize(self):
        self.num_joints = len(self.motor_configs)
        logger.info("Initializing mock motor reader.")
    
    def get_state(self):
        return np.zeros(self.num_joints)

    def stop(self):
        pass


if __name__ == "__main__":
    joint_config = [
        MotorConfig(joint_name="joint_1", port="port_1", id=1, sign=1, zero_point=0, scale=1, ref_point=0),
        MotorConfig(joint_name="joint_2", port="port_2", id=2, sign=1, zero_point=0, scale=1, ref_point=0),
    ]
    mock_motor_driver = MockMotorReader(ids=[1, 2], default_output=[0, 0])
    mock_motor_driver.initialize(joint_config)
    print(mock_motor_driver.get_state())
