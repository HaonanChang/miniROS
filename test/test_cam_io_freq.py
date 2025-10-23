import time
import os
from mini_ros.devices.cameras.rs_camera import RSCamera, RSCameraConfig
from mini_ros.test.cam_frequency_test import MultiThreadCameraFrequencyTest


if __name__ == "__main__":
    data_upload_dir = "./tmp_data"
    os.makedirs(data_upload_dir, exist_ok=True)
    camera = RSCamera(driver_config=RSCameraConfig(
        name="camera-left",
        device_id="315122271722",
        width=1280,
        height=720,
        fps=30,
        save_depth=False,
        save_format="jpg",
        flip=False,
        auto_exposure=True,
        auto_exposure_limit=False,
        manual_exposure_value=10000,
        ae_max_exposure=1000,
        jpeg_quality=95,
        data_upload_dir=data_upload_dir,
    ))
    camera.initialize()
    for iter in range(5):
        camera.start()
        for i in range(10):
            print(f"{iter}: Writing data {i}")
            time.sleep(1)
        camera.pause()
        time.sleep(1)
    camera.stop()