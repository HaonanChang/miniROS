import time
import os
import random
import pathlib
from loguru import logger
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.lang_util import LangUtil
from mini_ros.devices.cameras.rs_camera import RSCamera, RSCameraConfig
from mini_ros.test.cam_frequency_test import MultiThreadCameraFrequencyTest


def move_tmp_file(goal_folder: str, camera: RSCamera):
    target_path = pathlib.Path(goal_folder) / f"{camera.name}.jsonl"
    logger.info(f"Moving {camera.tmp_file} to {target_path}")
    # Move the file to the target folder
    os.replace(camera.tmp_file, str(target_path))


if __name__ == "__main__":
    data_upload_dir = "./tmp_data"
    device_ids = {
        "top_camera": "341522302010",
        "left_camera": "315122271722",
        "right_camera": "315122271608",
    }

    os.makedirs(data_upload_dir, exist_ok=True)
    cameras = []
    for name, device_id in device_ids.items():
        camera = RSCamera(driver_config=RSCameraConfig(
            name=name,
            device_id=device_id,
            width=640,
            height=480,
            fps=60,
        ))
        camera.initialize()
        cameras.append(camera)

    time.sleep(3)  # Wait for cameras to initialize

    for iter in range(2):
        goal_folder = os.path.join(data_upload_dir, TimeUtil.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(goal_folder, exist_ok=True)
        for camera in cameras:
            camera.start()
            camera.start_record(episode_name=f"e_{iter:06d}")
            print(f"Current file: {camera.tmp_file}")
        for i in range(5):
            print(f"{iter}: Writing data {i}")
            time.sleep(1)
        for camera in cameras:
            camera.stop_record()
            camera.pause()
        time.sleep(1)
        
        # # Move tmp files to goal folder
        # for camera in cameras:
        #     move_tmp_file(goal_folder, camera)
    # Stop all cameras
    for camera in cameras:
        camera.stop()