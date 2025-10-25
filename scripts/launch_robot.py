"""
Launch robot on web.
"""
import os
import time
from typing import List
import threading
import numpy as np
from loguru import logger
from mini_ros.utils.rate_limiter import RateLimiterSync
from mini_ros.common.state import RobotAction
from mini_ros.system.robot_frequency_test import  create_timed_joint_cmds_traj, draw_compare_fig
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.io.recorder import EpisodeRecorder, RecorderConfig
from mini_ros.wrapper.multi_robot import MultiRobotSystem
from mini_ros.devices.cameras.rs_camera import RSCamera, RSCameraConfig
from mini_ros.wrapper.parallel_robot import ParallelRobotConfig, WebParallelRobotMT
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.lang_util import LangUtil
from mini_ros.network.network_queue import One2ManyRecver
from typing import Dict


if __name__ == "__main__":
    data_upload_dir = "./tmp_data"
    os.makedirs(data_upload_dir, exist_ok=True)
    
    robot_sys = MultiRobotSystem(
        devices=[
            RSCamera(RSCameraConfig(name="top_camera", device_id="341522302010", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            RSCamera(RSCameraConfig(name="left_camera", device_id="315122271722", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            RSCamera(RSCameraConfig(name="right_camera", device_id="315122271608", width=640, height=480, fps=60, data_upload_dir=data_upload_dir)),
            MarvinRobot(MarvinRobotConfig(name="marvin")),
            PikaGripper(PikaGripperConfig(name="pika-0", port="/dev/ttyUSB0", read_size=0)),
            PikaGripper(PikaGripperConfig(name="pika-1", port="/dev/ttyUSB1", read_size=0)),
            EpisodeRecorder(RecorderConfig(name="marvin", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
            EpisodeRecorder(RecorderConfig(name="pika-0", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
            EpisodeRecorder(RecorderConfig(name="pika-1", fps=60, max_episode_length=200, data_root_dir=data_upload_dir)),
        ]
    )
    sys_config = ParallelRobotConfig(
        control_freqs={
            "marvin": 60,
            "pika-0": 60,
            "pika-1": 60,
        },
        read_freqs={
            "marvin": 200,
            "pika-0": 200,
            "pika-1": 200,
            "top_camera": 60,
            "left_camera": 60,
            "right_camera": 60,
        },
        episode_length=0,
    )

    # def update_config_callback():
    web_queues = []
    web_queues.append(One2ManyRecver(name="gello", port=5555, data_type="dict"))

    parallel_robot = WebParallelRobotMT(robot_sys, web_queues, sys_config)
    parallel_robot.initialize()

    export_folder = os.path.join(data_upload_dir, f"web_robot")
    os.makedirs(export_folder, exist_ok=True)
    
    try:
        parallel_robot.start()
        parallel_robot.join()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt (Ctrl+C), shutting down...")
        parallel_robot.stop()
        parallel_robot.join()
        logger.info("Robot system stopped successfully")
