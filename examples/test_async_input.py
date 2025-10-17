import os
import yaml
import asyncio
from loguru import logger
from typing import Dict, Any, List
from mini_ros.inputs.async_input import AsyncInput
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.device import Reader, motor_config_from_json


def get_reader(reader_type: str, include_names: List[str] = []):
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if reader_type == "feetech":
        from mini_ros.devices.motors.feetech import FeetechReader
        reader = FeetechReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_feetch.yaml")
    elif reader_type == "dynamixel":
        from mini_ros.devices.motors.dynamixel import DynamixelReader
        reader = DynamixelReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_dynamixel.yaml")
    elif reader_type == "mock":
        from mini_ros.devices.motors.mock_motor import MockMotorReader
        reader = MockMotorReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
    elif reader_type == "encoder":
        from mini_ros.devices.motors.encoder import EncoderReader
        reader = EncoderReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
    elif reader_type == "record3d":
        from mini_ros.devices.trackers.record3d_tracker import Record3DTrackerReader
        reader = Record3DTrackerReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
        joint_config_file = None
    else:
        raise ValueError(f"Invalid reader type: {reader_type}")
    if joint_config_file is not None:
        with open(joint_config_file, "r") as f:
            joint_config = yaml.load(f, Loader=yaml.FullLoader)
    else:
        joint_config = None
    if include_names:
        joint_config = motor_config_from_json(joint_config, include_names=include_names)
    return reader, joint_config

async def test_async_input(readers: Dict[str, Reader], joint_configs: Dict[str, Any], polling_rate: Dict[str, float]):
    async_input = AsyncInput(readers, joint_configs, polling_rate=polling_rate)
    await async_input.initialize()
    await async_input.start()
    await async_input.start_record()
    for i in range(500):
        logger.info(f"Reading {i} times")
        now = TimeUtil.now()
        # left_qpos = await async_input.get_state("encoder_left")
        record3d_qpos = await async_input.get_state("record3d")
        # logger.info(f"left_qpos: {left_qpos}, record3d_qpos: {record3d_qpos}")
        logger.info(f"Time: {1000.0 / TimeUtil.get_elapsed_time_ms(now)}Hz for reading")
    await async_input.stop()


if __name__ == "__main__":
    encoder_reader, left_joint_config = get_reader("encoder", include_names=[f"joint_{i}" for i in range(7)])
    encoder_reader, right_joint_config = get_reader("encoder", include_names=[f"joint_{i + 7}" for i in range(7)])
    record3d_reader, _ = get_reader("record3d")

    reader_dict = {
        # "encoder_left": encoder_reader,
        # "encoder_right": encoder_reader,
        "record3d": record3d_reader
    }
    reader_config_dict = {
        # "encoder_left": left_joint_config,
        # "encoder_right": right_joint_config,
        "record3d": None
    }
    polling_rate_dict = {
        # "encoder_left": 75,
        # "encoder_right": 75,
        "record3d": 30
    }
    asyncio.run(test_async_input(reader_dict, reader_config_dict, polling_rate_dict))