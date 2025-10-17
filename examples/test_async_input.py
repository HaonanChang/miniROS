from mini_ros.inputs.async_input import AsyncInput
import asyncio
import os
from typing import Dict, Any, List
from mini_ros.inputs.drivers.motor_driver import MotorDriver, MotorDriverAsyncWrapper, motor_config_from_json
from loguru import logger
import yaml
from mini_ros.utils.time_util import TimeUtil


def get_driver(driver_type: str, include_names: List[str] = []):
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
    joint_config = motor_config_from_json(joint_config, include_names=include_names)
    return driver, joint_config


async def test_async_input(drivers: Dict[str, MotorDriver], joint_configs: Dict[str, Any], driver_rate: Dict[str, int]):
    async_input = AsyncInput(drivers, joint_configs, driver_rate=driver_rate)
    await async_input.initialize(joint_configs)
    await async_input.start()
    await async_input.start_record()
    for i in range(1000):
        now = TimeUtil.now()
        left_qpos = await async_input.get_state("encoder_left")
        logger.info(f"Time: {1000.0 / TimeUtil.get_elapsed_time_ms(now)}Hz for reading")
        # right_qpos = await async_input.get_state("encoder_right")
        # logger.info(f"right_qpos: {right_qpos}")
        # await asyncio.sleep(0.01)
    await async_input.stop()


if __name__ == "__main__":
    encoder_driver, left_joint_config = get_driver("encoder", include_names=[f"joint_{i}" for i in range(7)])
    encoder_driver, right_joint_config = get_driver("encoder", include_names=[f"joint_{i + 7}" for i in range(7)])
    asyncio.run(test_async_input({"encoder_left": encoder_driver, "encoder_right": encoder_driver}, {"encoder_left": left_joint_config, "encoder_right": right_joint_config}, driver_rate={"encoder_left": 75, "encoder_right": 75}))

    # encoder_driver, left_joint_config = get_driver("encoder", include_names=[f"joint_{i}" for i in range(7)])
    # asyncio.run(test_async_input({"encoder_left": encoder_driver}, {"encoder_left": left_joint_config, }, driver_rate={"encoder_left": 150}))

    # encoder_driver, right_joint_config = get_driver("encoder", include_names=[f"joint_{i + 7}" for i in range(7)])
    # asyncio.run(test_async_input({"encoder_right": encoder_driver}, {"encoder_right": right_joint_config}, driver_rate={"encoder_right": 150}))