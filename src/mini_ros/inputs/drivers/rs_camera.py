import asyncio
import base64
import copy
import json
import pathlib
import pickle
import random
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple, Union

import aiofiles
import aiofiles.os
import numpy as np
import pyrealsense2 as rs2

from mini_ros.inputs.polling_input import PollingInput
from mini_ros.common.robot_types import RGBDCameraOutput
from loguru import logger
from mini_ros.common.state import RecordState
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.lang_util import LangUtil
from mini_ros.utils.time_util import TimeUtil




class RSCamera(PollingInput):
    """
    A class to represent a Intel Realsense camera.
    """

    # Class variable for fetching device_id
    fetch_id_mutex: asyncio.Lock = asyncio.Lock()

    def __init__(
        self,
        device_id: str = None,
        name: str = None,
        save_color=True,
        save_depth=False,
        width=1280,
        height=720,
        fps=30,
        sample_width=None,
        sample_height=None,
        save_format="jpg",
        flip=False,
        auto_exposure=True,
        auto_exposure_limit=False,
        manual_exposure_value=10000,
        ae_max_exposure=1000,
        jpeg_quality=95,
    ):

        # Pass parameters
        self._save_format = save_format
        self._save_color = save_color
        self._save_depth = save_depth
        self.sample_width = sample_width
        self.sample_height = sample_height
        self.width = width
        self.height = height
        self.fps = fps
        self.param_name = name
        self.param_device_id = device_id
        self._device_id = None
        self._flip = flip

        self._rs_config = None
        self._rs_pipeline = None
        self._depth_frames = []
        self._color_frames = []
        self._save_mutex = asyncio.Lock()
        self._is_saving = False
        self._name = None
        self._io_mutex = asyncio.Lock()
        self._auto_exposure = auto_exposure
        self._auto_exposure_limit = auto_exposure_limit
        self._ae_max_exposure = ae_max_exposure
        self._manual_exposure_value = manual_exposure_value

        self._jpeg_quality = jpeg_quality
        self._data_upload_dir = "/tmp"

        # File handlers for saving frames
        self._fh_color = None
        self._fh_depth = None

        # If either sample_width or sample_height is not set, use the stream size
        if not self.sample_width or not self.sample_height:

            self.sample_width = 640
            self.sample_height = 360

            logger.debug(
                self.__class__.__name__,
                f"sample_width or sample_height not set by config passed in, default sample size {self.sample_width}x{self.sample_height} will be prioritized",
                require_stack=False,
            )

    def is_fliped(self) -> bool:
        """
        Whether this camera is producing 180-degree rotated frames
        """

        return self._flip

    @classmethod
    async def get_first_device_id(cls) -> str:
        """
        Get the first available device_id from all available cameras

        Returns:
            The first available device_id as a string
        """

        async with cls.fetch_id_mutex:
            if cls._available_device_ids is None:

                def get_devices():
                    devices = rs2.context().query_devices()

                    for d in devices:
                        d.hardware_reset()

                    return devices

                # Make device_id popping more deterministic by sorting
                avail = sorted(
                    [
                        d.get_info(rs2.camera_info.serial_number)
                        for d in await AsyncUtil.run_blocking_as_async(get_devices)
                    ]
                )

                logger.info(f"Available device ids: {avail}")

                if len(avail) == 0:
                    raise Exception("No cameras available!")

                cls._available_device_ids = avail

            elif len(cls._available_device_ids) == 0:
                raise Exception("All cameras are initialized!")

        return cls._available_device_ids.pop(0)

    async def _do_start_record(self):
        """
        Start recording the camera.
        """

        # Save temporary files to a mounted path (In this case, the same path that the data will be at)
        work_dir = (
            pathlib.Path(
                self._data_upload_dir
            )
            / ".tmp/"
        )
        work_dir.mkdir(parents=True, exist_ok=True)

        if self._save_color:
            self._tmp_path_color = work_dir / f"{LangUtil.make_uid()}.jsonl"

            logger.info(
                self.__class__.__name__,
                await self.name(),
                f"Saving color frames to {self._tmp_path_color} as temporary file",
            )

            async with self._io_mutex:
                self._fh_color = await aiofiles.open(self._tmp_path_color, "w")

        if self._save_depth:
            self._tmp_path_depth = work_dir / f"{LangUtil.make_uid()}-depth.jsonl"

            logger.info(
                self.__class__.__name__,
                await self.name(),
                f"Saving depth frames to {self._tmp_path_depth} as temporary file",
            )

            async with self._io_mutex:
                self._fh_depth = await aiofiles.open(self._tmp_path_depth, "w")

    async def _do_discard_data(self):
        """
        Discard the data from the camera.
        """

        # TODO: Do something here

        if (
            self._save_color
            and self._fh_color is not None
            and not self._fh_color.closed
        ):

            await self._fh_color.close()
            # Remove the file
            await aiofiles.os.remove(self._tmp_path_color)

        if (
            self._save_depth
            and self._fh_depth is not None
            and not self._fh_depth.closed
        ):

            await self._fh_depth.close()
            await aiofiles.os.remove(self._tmp_path_depth)

    async def device_id(self) -> str:
        """
        Get the device_id of the camera. Will try to find one if not provided.
        """

        if self._device_id is None:

            if self.param_device_id is None:
                logger.warning(
                    f"No device_id provided for RSCamera '{self.param_name}', trying to find one..."
                )
                self._device_id = await self.get_first_device_id()
            else:
                self._device_id = self.param_device_id

        return self._device_id

    async def name(self) -> str:
        """
        Get the name of the camera.
        """

        if self._name is None:

            if self.param_name is None:
                self._name = f"RSCamera_{await self.device_id()}"
            else:
                self._name = self.param_name

        return self._name

    async def display_name(self) -> str:
        return f"RSCamera-{await self.name()}"

    def _is_crash_on_read_fail(self) -> bool:
        """
        Check if the input device should crash on read failure
        """

        return True

    async def _do_initialize(self):

        self._depth_frames = []
        self._color_frames = []
        width = self.width
        height = self.height
        # device_id = LangUtil.get_dict_value(self.config, "device_id", None)

        if self._save_depth:
            mode_map = {
                rs2.stream.color: rs2.format.bgr8,
                rs2.stream.depth: rs2.format.z16,
            }
        else:
            mode_map = {rs2.stream.color: rs2.format.bgr8}

        self._rs_pipeline = rs2.pipeline()
        self._rs_config = rs2.config()

        logger.debug(self.__class__.__name__, await self.name(), f"Enabling device")

        # Read device_id or grab one to enable the device
        self._rs_config.enable_device(await self.device_id())

        # Enable both depth and color streams
        # D405 MUST enable depth stream for AE to work, even if we don't save it
        #FIXME: This is not true, need for more testing
        for stream, color in mode_map.items():

            logger.debug(
                self.__class__.__name__,
                await self.name(),
                f"Enabling stream {stream} with width {width}, height {height}, color {color}, fps {self.fps}",
            )

            self._rs_config.enable_stream(stream, width, height, color, self.fps)
        try:
            logger.debug(
                self.__class__.__name__,
                await self.name(),
                f"Starting pipeline",
            )
            # Start Camera Pipeline
            self._pipeline = await AsyncUtil.run_blocking_as_async(
                self._rs_pipeline.start, self._rs_config
            )

            self._sensor = self._pipeline.get_device().first_depth_sensor()

            logger.debug(
                "Supported Options: ",
                self._sensor.get_supported_options(),
            )

            logger.debug(
                self.__class__.__name__,
                await self.name(),
                "Auto Exposure Range: ",
                self._sensor.get_option_range(rs2.option.auto_exposure_limit),
            )

            logger.debug(
                self.__class__.__name__,
                await self.name(),
                "Auto Exposure Limit Toggle: ",
                self._sensor.get_option_range(rs2.option.auto_exposure_limit_toggle),
            )

            logger.debug(
                self.__class__.__name__,
                await self.name(),
                "Frames Queue Size Range: ",
                self._sensor.get_option_range(rs2.option.frames_queue_size),
            )

            if self._auto_exposure:
                self._sensor.set_option(
                    rs2.option.enable_auto_exposure,
                    True,
                )

                if self._auto_exposure_limit:
                    self._sensor.set_option(
                        rs2.option.auto_exposure_limit_toggle,
                        True,
                    )

                    self._sensor.set_option_value(
                        rs2.option.auto_exposure_limit,
                        float(self._ae_max_exposure),
                    )
            else:
                self._sensor.set_option(
                    rs2.option.enable_auto_exposure,
                    False,
                )

                self._sensor.set_option_value(
                    rs2.option.exposure,
                    float(self._manual_exposure_value),
                )

            # Configure frame buffer size
            # Configure frame buffer size to 8
            self._sensor.set_option(rs2.option.frames_queue_size, 8)
            # try:
            #     # Set the frames queue size option to 8

            #     logger.debug(
            #         self.__class__.__name__,
            #         await self.name(),
            #         "Set frames queue size to 8",
            #     )
            # except Exception as e:
            #     logger.warning(
            #         self.__class__.__name__,
            #         await self.name(),
            #         f"Failed to set frames queue size: {e}",
            #     )
            # logger.debug(
            #     self.__class__.__name__,
            #     await self.name(),
            #     f"Pipeline started!",
            # )

            logger.info(
                self.__class__.__name__,
                await self.name(),
                "Online and ready to record!",
            )

        except Exception as e:
            logger.error(
                f"Error starting pipeline for {await self.name()}", require_stack=False
            )
            logger.error(e)
            raise e

    def is_no_wait(self) -> bool:

        # We want to read as fast as possible. wait_for_frames is blocking us anyway
        return True

    def sample_size(self) -> Optional[Tuple[int, int]]:
        """
        Get the image sample size from the config.
        This used to be a parameter of _read() in data-collection, but since it doesn't change / mostly passed as None,
        this is now treated as a config parameter.

        Returns:
            The sample size as a tuple of (width, height)
        """

        if not self.sample_width or not self.sample_height:  # Invalid Configuration
            return self.stream_size()

        return (self.sample_width, self.sample_height)

    def stream_size(self) -> Optional[Tuple[int, int]]:
        """
        Get the stream size from the config.
        """

        return (self.width, self.height)

    async def _get_processed_frame(
        self, frame: rs2.frame, is_depth: bool
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the processed frame (Resized, Flipped, colored if depth) from the input frame (Realsense frame).
        This method is async-ready. OpenCV calls are moved to another thread.

        Args:
            frame: The input frame
            is_depth: Whether the frame is a depth frame (Because there's an indexing operation on the depth frame)

        Returns:
            Processed Frames. First frame in original size, second frame in resized size
        """

        import cv2

        def blocking_func():
            img = np.asanyarray(frame.get_data())

            # Flip the image if needed
            if self.is_fliped():
                img = cv2.rotate(img, cv2.ROTATE_180)

            # Transform depth frame to color frame for visualization
            # TODO: This is GPT-generated code. Test this
            # Prompt: Imagine I am using pyrealsense2 to capture depth image in z16 format. How to convert this to JPEG with opencv?
            if is_depth:
                depth_image_normalized = cv2.normalize(
                    img, None, 0, 255, cv2.NORM_MINMAX
                )
                depth_image_normalized = np.uint8(depth_image_normalized)

                # Apply a colormap for better visualization
                img = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

            resized_img = copy.deepcopy(img)

            # Resize the image if needed
            if (
                self.sample_size() is not None  # Sample Size is set
                and self.sample_size()
                != self.stream_size()  # Sample Size != Stream Size
            ):
                resized_img = cv2.resize(resized_img, self.sample_size())

            return img, resized_img

        return await AsyncUtil.run_blocking_as_async(blocking_func)

    @classmethod
    async def _encode_frame(cls, frame: np.ndarray, save_fmt: str = "jpg", jpeg_quality: int = 95) -> bytes:
        """
        Encode a OpenCV frame to a bytes object
        """

        import cv2

        # If out of bounds, set to default
        if jpeg_quality < 30 or jpeg_quality > 100:
            jpeg_quality = 95

        return (
            await AsyncUtil.run_blocking_as_async(
                cv2.imencode,
                # TODO: Update the image quality as save format changes
                f".{save_fmt}",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
            )
        )[1].tobytes()

    async def _walk_save_pipeline(
        self, frame: rs2.frame, is_depth: bool, ts_system: float, ts_realsense: float
    ) -> Optional[bytes]:

        # Convert Realsense frame to OpenCV frame
        processed, resized = await self._get_processed_frame(frame, is_depth)

        # Encode the frame from OpenCV frames to bytes (JPEG)

        encoded = await self._encode_frame(processed, self._save_format, self._jpeg_quality)
        encoded_resized = await self._encode_frame(resized, self._save_format, self._jpeg_quality)

        if self.record_state == RecordState.ACTIVE:

            target_arr = self._color_frames if not is_depth else self._depth_frames
            # Encode the frame from OpenCV frames to bytes

            async def save_frame_to_ram():

                payload = json.dumps(
                    {
                        "timestamp": ts_realsense,
                        "timestamp_recv": ts_system,
                        "image": await LangUtil.encode_to_base64(encoded),
                    }
                )

                try:
                    async with self._io_mutex:
                        if (
                            is_depth
                            and self._fh_depth is not None
                            and not self._fh_depth.closed
                        ):
                            await self._fh_depth.write(payload + "\n")
                        elif (
                            not is_depth
                            and self._fh_color is not None
                            and not self._fh_color.closed
                        ):
                            await self._fh_color.write(payload + "\n")
                except Exception as e:
                    # Because "write to a closed file".
                    # But it's not really a problem for our case
                    pass

            # OpenCV is blocking and CPU intensive. Avoid blocking the main thread
            # This will REQUIRE the get_<type>_frames() APIs to sort the frames by timestamp before returning
            AsyncUtil.detach_coroutine(save_frame_to_ram())

        return encoded_resized

    async def _request_input(self):

        try:
            frames = await AsyncUtil.run_blocking_as_async(
                self._rs_pipeline.wait_for_frames
            )
        except Exception as e:
            logger.error(
                self.__class__.__name__,
                await self.name(),
                f"Error waiting for frames",
                "" if len(e.args) == 0 else e.args[0],
                require_stack=False,
            )
            raise e

        ts_system = TimeUtil.now().timestamp()

        # Get realsense's timestamp (Ground Truth) in seconds
        ts_realsense = frames.get_timestamp() / 1000.0

        color_frame = None
        depth_frame = None

        if self._save_color:
            color_frame = await self._walk_save_pipeline(
                frames.get_color_frame(), False, ts_system, ts_realsense
            )

        if self._save_depth:
            depth_frame = await self._walk_save_pipeline(
                frames.get_depth_frame(), True, ts_system, ts_realsense
            )

        return RGBDCameraOutput(
            color_frame=color_frame,
            depth_frame=depth_frame,
        )

    async def save_data(self, path: Optional[str] = None):
        """
        Save the frames to storage.
        Args:
            folder_path: The path to save the frames to. If None, a random folder will be created. (BUT YOU SHOULD PASS ONE IN)
        """

        if self.is_saving():
            return

        self._is_saving = True

        # Stop record to make sure buffer does not get larger
        await self.stop_record()

        if path is None:
            random_id = "".join([f"{random.randint(1, 10)}" for _ in range(6)])
            timestamp = TimeUtil.now().strftime("%Y%m%d_%H%M%S")
            path = LangUtil.get_abs_path(f"data/{timestamp}_{random_id}")
        else:
            path = pathlib.Path(path)

        # make sure target folder exists
        path.mkdir(parents=True, exist_ok=True)

        async with self._io_mutex:
            if self._save_color and self._fh_color is not None:

                await self._fh_color.close()

                target_path = path / f"{await self.name()}.jsonl"

                logger.verbose(
                    self.__class__.__name__,
                    await self.name(),
                    f"Saving color frames to {target_path}",
                )

                # Move the file to the target folder
                await aiofiles.os.replace(self._tmp_path_color, target_path)

            if self._save_depth and self._fh_depth is not None:

                await self._fh_depth.close()

                target_path = path / f"{await self.name()}-depth.jsonl"

                logger.verbose(
                    self.__class__.__name__,
                    await self.name(),
                    f"Saving depth frames to {target_path}",
                )

                # Move the file to the target folder
                await aiofiles.os.replace(self._tmp_path_depth, target_path)

        self._is_saving = False

    def is_saving(self) -> bool:
        """
        Whether the camera is saving frames to storage.
        """

        return self._is_saving

    async def _do_stop(self) -> int:
        if self._rs_pipeline is not None:
            await AsyncUtil.run_blocking_as_async(self._rs_pipeline.stop)
            await AsyncUtil.run_blocking_as_async(self._rs_pipeline.stop)
