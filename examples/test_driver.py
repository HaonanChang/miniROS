"""
Test different drivers
"""
import asyncio
import os
from typing import Dict, Any
from mini_ros.inputs.drivers.motor_driver import MotorDriver, MotorDriverAsyncWrapper, motor_config_from_json
from loguru import logger
import yaml


def get_driver(driver_type: str):
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if driver_type == "feetech":
        from mini_ros.inputs.drivers.feetech_driver import FeetechDriver
        driver = FeetechDriver()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_feetch.yaml")
    elif driver_type == "dynamixel":
        from mini_ros.inputs.drivers.dynamixel_driver import DynamixelGello
        driver = DynamixelGello()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_dynamixel.yaml")
    elif driver_type == "mock":
        from mini_ros.inputs.drivers.mock_driver import MockMotorDriver
        driver = MockMotorDriver()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
    elif driver_type == "encoder":
        from mini_ros.inputs.drivers.encoder_driver import EncoderDriver
        driver = EncoderDriver()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
    else:
        raise ValueError(f"Invalid driver type: {driver_type}")
    with open(joint_config_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)
    return driver, joint_config


async def test_driver(driver: MotorDriver, joint_config: Dict[str, Any]):
    driver = MotorDriverAsyncWrapper(driver)
    await driver.initialize(motor_config_from_json(joint_config))
    while True:
        qpos = await driver.get_state()
        logger.info(qpos)
        await asyncio.sleep(0.01)


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    driver_type = "encoder"
    driver, joint_config = get_driver(driver_type)
    asyncio.run(test_driver(driver, joint_config))