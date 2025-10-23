import zlib
from mini_ros.common.device import Camera
from dataclasses import dataclass
import numpy as np
from typing import Any
import pyrealsense2 as rs2
import os
from loguru import logger
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.rate_limiter import RateLimiterSync
from queue import Queue
import cv2
import threading
import time
import json
import struct
import base64
from mini_ros.common.state import CameraData
from mini_ros.utils.lang_util import LangUtil


@dataclass
class RSCameraConfig:
    device_id: str
    name: str
    width: int
    height: int
    fps: int
    save_depth: bool = False
    save_format: str = "jpg"
    flip: bool = False
    auto_exposure: bool = True
    auto_exposure_limit: bool = False
    manual_exposure_value: int = 10000
    ae_max_exposure: int = 1000
    jpeg_quality: int = 95
    # Resizing size
    resize_width: int = None
    resize_height: int = None
    # Save options
    data_upload_dir: str = "/tmp"
    buffer_size: int = 1
    # Automatically run save_data in backend loop
    is_auto_save: bool = True


class RSCamera(Camera):

    name: str = "rs_camera"
    """
    A class to represent a Intel Realsense camera.
    """
    def __init__(self, driver_config):
        #  Parse driver config
        self.device_id = driver_config.device_id
        self.name = driver_config.name
        self.save_depth = driver_config.save_depth
        self.width = driver_config.width
        self.height = driver_config.height
        self.fps = driver_config.fps
        self.save_format = driver_config.save_format
        assert self.save_format in ["jpg", "jpeg"], "Only jpg and jpeg are supported"
        self.flip = driver_config.flip
        self.auto_exposure = driver_config.auto_exposure
        self.auto_exposure_limit = driver_config.auto_exposure_limit
        self.manual_exposure_value = driver_config.manual_exposure_value
        self.ae_max_exposure = driver_config.ae_max_exposure
        self.jpeg_quality = driver_config.jpeg_quality
        self.resize_width = driver_config.resize_width
        self.resize_height = driver_config.resize_height
        self.data_upload_dir = driver_config.data_upload_dir
        self.read_rate_limiter = RateLimiterSync(self.fps)
        # Saver & Reader rate limiter should be the same
        self.auto_save_rate_limiter = RateLimiterSync(self.fps)
        self._is_auto_save = driver_config.is_auto_save
        # Data buffer
        self._data_buffer: Queue[CameraData] = Queue(maxsize=driver_config.buffer_size)
        # Event for data buffer
        self._active_event = threading.Event()
        self._connect_event = threading.Event()
        # Handle mutex
        self._io_mutex = threading.Lock()

    def is_active(self) -> bool:
        return self._active_event.is_set()

    def is_alive(self) -> bool:
        return self._connect_event.is_set()

    def initialize(self, driver_config: Any = None):
        """
        Build communications & setup camera config
        """
        self._rs_pipeline = rs2.pipeline()
        self._rs_config = rs2.config() 

        logger.info(self.name, f"Enabling device")
        self._rs_config.enable_device(self.device_id)

        # Enable color
        self._rs_config.enable_stream(rs2.stream.color, self.width, self.height, rs2.format.bgr8, self.fps)

        # Enable depth
        if self.save_depth:
            self._rs_config.enable_stream(rs2.stream.depth, self.width, self.height, rs2.format.z16, self.fps)

        # Start pipeline
        self._pipeline = self._rs_pipeline.start(self._rs_config)
        self._sensor = self._pipeline.get_device().first_depth_sensor()
        self._connect_event.set()

        # Configure auto exposure
        if self.auto_exposure:
            self._sensor.set_option(rs2.option.enable_auto_exposure, True)
            if self.auto_exposure_limit:
                self._sensor.set_option(rs2.option.auto_exposure_limit_toggle, True)
                self._sensor.set_option(rs2.option.auto_exposure_limit, self.ae_max_exposure)
        else:
            self._sensor.set_option(rs2.option.enable_auto_exposure, False)
            self._sensor.set_option(rs2.option.exposure, self.manual_exposure_value)

        # Configure frame buffer size
        self._sensor.set_option(rs2.option.frames_queue_size, 8)

        logger.info(self.name, f"Initialized camera")

        # Start read loop
        self._read_thread = threading.Thread(target=self.read_loop)
        self._read_thread.start()
        if self._is_auto_save:
            self._auto_save_thread = threading.Thread(target=self.auto_save_loop)
            self._auto_save_thread.start()
        time.sleep(0.25)
    
    def get_frame(self) -> np.ndarray:
        frames = self._rs_pipeline.wait_for_frames()
        return frames

    def start(self):
        if not self.is_alive():
            # Can't be double started
            logger.warning("Camera is not connected, can't be started")
            return

        # Drop old frame
        while not self._data_buffer.empty():
            self._data_buffer.get()

        # Open file handler for data
        tmp_dir = f"{self.data_upload_dir}/.tmp"
        self.tmp_file = f"{tmp_dir}/{LangUtil.make_uid()}.jsonl"
        os.makedirs(tmp_dir, exist_ok=True)
        with self._io_mutex:
            self._fh = open(self.tmp_file, "w")
        self._active_event.set()
        logger.info(f"Started camera: {self.name}")
        
    def pause(self):
        if not self.is_active():
            # Can't be double paused
            return
        self._active_event.clear()
        logger.info(f"Pausing camera: {self.name}: Active: {self.is_active()}, Alive: {self.is_alive()}")
        with self._io_mutex:
            # Clear the buffer
            self._fh.close()
            self._fh = None
        
    def stop(self):
        if not self.is_alive():
            # Can't be double stopped
            logger.warning("Camera is not connected, can't be stopped")
            return
        self._active_event.clear()
        self._connect_event.clear()
        with self._io_mutex:
            if self._fh is not None:
                logger.info(f"File is not yet closed. Closing file handler: {self.tmp_file}...")
                self._fh.close()
                self._fh = None

        self._rs_pipeline.stop()
        self._read_thread.join()
        if self._is_auto_save:
            self._auto_save_thread.join()

    def read_loop(self):
        while self.is_alive():
            try:
                self.read_rate_limiter.wait_for_tick()
                frame = self.get_frame()
                self.process_frame(frame)
            except Exception as e:
                logger.error(f"Error in read loop: {e}")
                break
    
    def process_frame(self, frames: rs2.frame) -> np.ndarray:
        """
        Encode frame object to goal format
        """
        color_frame = frames.get_color_frame()
        timestamp_in_chip = color_frame.get_timestamp() / 1000.0  # ms to s
        timestamp_in_system = TimeUtil.now().timestamp()
        
        img = np.asanyarray(color_frame.get_data())
        if self.flip:
            img = cv2.rotate(img, cv2.ROTATE_180)
        if self.resize_width and self.resize_height:
            img = cv2.resize(img, (self.resize_width, self.resize_height))
        color_image = self._jpeg_compress(img, self.save_format, self.jpeg_quality)

        # Encode depth image
        if self.save_depth:
            depth_frame = frames.get_depth_frame()
            if self.resize_width and self.resize_height:
                depth_frame = cv2.resize(depth_frame, (self.resize_width, self.resize_height), interpolation=cv2.INTER_NEAREST_EXACT)
            depth_image = self._zlib_compress(depth_frame.get_data())
        else:
            depth_image = None

        camera_data = CameraData(timestamp=timestamp_in_chip, timestamp_recv=timestamp_in_system, color_image=color_image, depth_image=depth_image, width=self.resize_width if self.resize_width else self.width, height=self.resize_height if self.resize_height else self.height)
        # Save to buffer
        if self._data_buffer.full():
            self._data_buffer.get_nowait()
        self._data_buffer.put_nowait(camera_data)

        return camera_data

    def _jpeg_compress(self, frame: np.ndarray, save_fmt: str = "jpg", jpeg_quality: int = 95) -> bytes:
        """
        Encode a OpenCV frame to a bytes object.
        Input: np.ndarray (H, W, 3)
        Output: bytes
        """
        # If out of bounds, set to default
        if jpeg_quality < 30 or jpeg_quality > 100:
            jpeg_quality = 95

        return cv2.imencode(f".{save_fmt}", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])[1].tobytes()
        
    def _zlib_compress(frame, compression_level=6):
        """
        Custom compression using zlib.
        Input: np.ndarray (H, W)
        Output: bytes
        """
        # Flatten the array and compress
        compressed = zlib.compress(frame.tobytes(), compression_level)
        # Add dimensions header for reconstruction
        header = struct.pack('II', frame.shape[1], frame.shape[0])
        return header + compressed

    ######################### Save related methods #########################
    def save_data(self, save_type: str = "color"):
        """
        Save the data to the given path
        """
        data_frame = self._data_buffer.get()
        # Serialize data to JSON
        payload = json.dumps(
            {
                "timestamp": data_frame.timestamp,
                "timestamp_recv": data_frame.timestamp_recv,
                "image": base64.b64encode(data_frame.color_image).decode("utf-8"),
                "depth": base64.b64encode(data_frame.depth_image).decode("utf-8") if data_frame.depth_image is not None else None,
            }
        )
        with self._io_mutex:
            if not self.is_active():
                return
            if self._fh is None:
                logger.error("File handler is not opened, can't save data")
                return
            self._fh.write(payload + "\n")
    
    def auto_save_loop(self):
        while self.is_alive():
            self.auto_save_rate_limiter.wait_for_tick()
            self.save_data()