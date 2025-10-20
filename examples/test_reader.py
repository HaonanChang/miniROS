"""
Test different drivers
"""
import asyncio
import os
import time
from typing import Dict, Any
from mini_ros.common.device import Reader, Streamer, motor_config_from_json
from mini_ros.devices.motors.feetech import FeetechReader
from mini_ros.devices.motors.dynamixel import DynamixelReader
from mini_ros.devices.motors.mock_motor import MockMotorReader
from mini_ros.devices.motors.encoder import EncoderReader
from mini_ros.devices.trackers.record3d_tracker import Record3DTracker
from loguru import logger
import yaml


def get_reader(reader_type: str):
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if reader_type == "feetech":
        reader = FeetechReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_feetch.yaml")
    elif reader_type == "dynamixel":
        reader = DynamixelReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_dynamixel.yaml")
    elif reader_type == "mock":
        reader = MockMotorReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
    elif reader_type == "encoder":
        reader = EncoderReader()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
    elif reader_type == "record3d":
        reader = Record3DTracker()
        joint_config_file = os.path.join(root_dir, "assets/gellos/joint_map_encoder.yaml")
        joint_config_file = None
    else:
        raise ValueError(f"Invalid reader type: {reader_type}")
    if joint_config_file is not None:
        with open(joint_config_file, "r") as f:
            joint_config = yaml.load(f, Loader=yaml.FullLoader)
    else:
        joint_config = None
    return reader, joint_config


def test_reader(reader: Reader, joint_config: Dict[str, Any]):
    reader.initialize(joint_config if joint_config is not None else None)
    while True:
        qpos = reader.get_state()
        logger.info(qpos)
        time.sleep(0.01)


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    driver_type = "record3d"
    reader, joint_config = get_reader(driver_type)
    test_reader(reader, joint_config)