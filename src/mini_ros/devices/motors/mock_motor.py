"""
General mocked driver, you can define the input you want.
"""

from typing import Optional, Sequence, List
import numpy as np
from loguru import logger
from mini_ros.common.device import MotorDevice, MotorConfig, motor_config_from_json


class MockMotorReader(MotorDevice):
    """
    A mock motor driver, you can define the angle you want
    """

    def __init__(
        self,
        ids: Sequence[int],
        name=None,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1000000,
        default_output: Sequence[int] = [],
    ):
        self._ids = ids
        self._port = port
        self._baudrate = baudrate
        self._name = name
        self._default_output = default_output

    def name(self) -> str:
        if self._name is not None:
            return self._name

    def initialize(self, joint_config: List[MotorConfig]):
        self.joint_config = joint_config
        self.num_joints = len(self.joint_config)
        logger.info("Initializing mock motor driver.")
    
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
