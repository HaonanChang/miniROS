import base64
import dataclasses
import json
import os
import pathlib
import shutil
import subprocess
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
from loguru import logger

from mini_ros.utils.lang_util import LangUtil
from mini_ros.utils.common_util import CommonUtil
from mini_ros.utils.system_util import SystemUtil
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.state import FrameTime


class VideoUtil:

    @classmethod
    def calculate_fps(cls, timestamps: List[float]):
        """
        Calculate the FPS of a list of timestamps.

        Args:
            timestamps: List of timestamps in nanoseconds

        Returns:
            FPS as a float
        """
        if len(timestamps) < 2:
            return 30  # Default to 30 if not enough data

        # Calculate time differences between consecutive frames
        time_diffs = np.diff(timestamps)
        avg_time_diff = np.mean(time_diffs)

        # Convert nanoseconds to seconds and calculate FPS
        fps = 1e9 / avg_time_diff
        return fps

    @classmethod
    def make_single_video(
        cls,
        raw_jsonl_path: pathlib.Path,
        output_video_path: pathlib.Path,
        start_ts_ns: int,
    ) -> List[FrameTime]:
        """
        [This is a thread-safe function]
        Make a single video from a raw JSONL file.

        Args:
            raw_jsonl_path: Path to the raw JSONL file
            output_video_path: Path to the output video file (This function also needs write access to the parent folder of the output video file)
            start_ts_ns: Start timestamp in nanoseconds

        Returns:
            List of FrameTime objects, which can be used to create a one-to-one mapping to the frames.
        """
        timestamps: List[FrameTime] = []

        start_ts_seconds = float(start_ts_ns) / 1e9
        # Make sure the temporary folder exists
        # UUIDv4 folder to avoiding collision on temp deletion
        temp_path = (
            pathlib.Path(output_video_path.parent) / f".frames_{LangUtil.make_uid()}"
        )
        temp_path.mkdir(parents=True, exist_ok=True)

        frame_count = 0

        with open(raw_jsonl_path, "r") as fh_input:
            for line in fh_input:
                if line.strip() == "":
                    continue

                entry = json.loads(line.strip())

                # These are in seconds. RDC outputs Unix timestamps, which is in seconds.
                ts = entry["timestamp"]
                ts_recv = entry["timestamp_recv"]

                # Trim movement -- Though, this will be set to the start of the video if trim is off
                # At which point this claude never runs, which is what we want
                if ts < start_ts_seconds:
                    continue

                # Save the timestamps to return to the caller -> caller needs it for MCAP writing, most likely
                timestamps.append(
                    FrameTime(
                        timestamp_ns=CommonUtil.timestamp_to_nanosec(ts),
                        timestamp_recv_ns=CommonUtil.timestamp_to_nanosec(ts_recv),
                    )
                )

                # Dump the image as-is (we don't need to resize or encode again)
                with open(
                    str((temp_path / f"frame_{frame_count:09d}.jpg").resolve()), "wb"
                ) as fh_output:
                    fh_output.write(base64.b64decode(entry["image"]))

                frame_count += 1

        if frame_count == 0:
            raise Exception(
                f"No frames found in {raw_jsonl_path}. Cannot generate video."
            )

        fps = cls.calculate_fps([t.timestamp_ns for t in timestamps])
        input_pattern = os.path.join(str(temp_path.resolve()), "frame_%09d.jpg")

        cmd = f"ffmpeg -y -framerate {fps} -i {input_pattern} -c:v libx264 -profile:v baseline -level 3.0 -pix_fmt yuv420p -preset medium -crf 23 {str(output_video_path.resolve())}"

        proc = SystemUtil.run_command(cmd)

        # After done, delete the temp folder
        SystemUtil.run_command(f"rm -rf {str(temp_path.resolve())}")

        if proc.returncode != 0:
            raise Exception(f"Failed to generate video: {proc.returncode}")

        return timestamps

    @classmethod
    def generate_video(cls, folder_path, cpu_only: bool = False):
        folder_name = os.path.basename(folder_path.rstrip("/"))

        top_camera_file = os.path.join(folder_path, "top_camera.jsonl")
        left_camera_file = os.path.join(folder_path, "left_camera.jsonl")
        right_camera_file = os.path.join(folder_path, "right_camera.jsonl")
        output_video_file = os.path.join(folder_path, f"{folder_name}.mp4")
        # tmp_video_file = os.path.join(folder_path, f"{folder_name}_tmp.mp4") # No longer needed

        temp_frames_folder = os.path.join(folder_path, "temp_frames")
        os.makedirs(temp_frames_folder, exist_ok=True)

        # First pass: collect timestamps and get first frame for dimensions
        top_camera_timestamps = []
        first_frame = None

        if os.path.exists(top_camera_file):
            with open(top_camera_file, "r") as f:
                for line in f:
                    if line.strip() == "":
                        continue
                    entry = json.loads(line.strip())
                    top_camera_timestamps.append(
                        CommonUtil.timestamp_to_nanosec(entry["timestamp"])
                    )
                    if first_frame is None:
                        first_frame = entry

        if first_frame is None:
            print(f"No frames found in {top_camera_file}. Cannot generate video.")
            if os.path.exists(temp_frames_folder):  # Clean up temp folder if created
                shutil.rmtree(temp_frames_folder)
            return

        fps = cls.calculate_fps(top_camera_timestamps)
        logger.info(f"Detected FPS: {fps}")

        # Get the frame size from the first image
        first_image = cv2.imdecode(
            np.frombuffer(base64.b64decode(first_frame["image"]), np.uint8),
            cv2.IMREAD_COLOR,
        )
        height, width, _ = first_image.shape

        # Ensure the directory for the output file exists (already done for parent, but good practice if output_video_file was elsewhere)
        os.makedirs(os.path.dirname(output_video_file), exist_ok=True)

        # Removed VideoWriter object creation:
        # fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # video_writer = cv2.VideoWriter(tmp_video_file, fourcc, fps, (width, height * 3))

        # Initialize iterators for left and right cameras
        left_iterator = None
        right_iterator = None
        left_current = None
        right_current = None

        if os.path.exists(left_camera_file):
            left_iterator = open(left_camera_file, "r")
            left_current = next(left_iterator, None)
            if left_current and left_current.strip():
                left_current = json.loads(left_current.strip())

        if os.path.exists(right_camera_file):
            right_iterator = open(right_camera_file, "r")
            right_current = next(right_iterator, None)
            if right_current and right_current.strip():
                right_current = json.loads(right_current.strip())

        frame_count = 0

        def decode_image(raw_data: Optional[Dict[str, Any]]) -> Optional[np.ndarray]:
            if raw_data is None:
                return None

            return cv2.imdecode(
                np.frombuffer(base64.b64decode(raw_data["image"]), np.uint8),
                cv2.IMREAD_COLOR,
            )

        # Process each frame
        if os.path.exists(top_camera_file):
            # Initialize default images
            left_image = None
            right_image = None
            blank_img = None
            with open(top_camera_file, "r") as f_top:
                for line in f_top:
                    if line.strip() == "":
                        continue

                    # Decode the top camera image
                    top_entry = json.loads(line.strip())
                    top_image = decode_image(top_entry)
                    # top_image = cv2.imdecode(
                    #     np.frombuffer(base64.b64decode(top_entry["image"]), np.uint8),
                    #     cv2.IMREAD_COLOR,
                    # )

                    if blank_img is None:
                        blank_img = np.zeros_like(top_image)

                    if (
                        left_image is None
                    ):  # Initialize black images if not yet available
                        left_image = blank_img
                    if right_image is None:
                        right_image = blank_img
                    timestamp_nanoseconds = CommonUtil.timestamp_to_nanosec(
                        top_entry["timestamp"]
                    )

                    # Process left camera
                    if left_iterator and left_current:
                        while (
                            left_current
                            and CommonUtil.timestamp_to_nanosec(
                                left_current["timestamp"]
                            )
                            <= timestamp_nanoseconds
                        ):
                            # left_image = cv2.imdecode(
                            #     np.frombuffer(
                            #         base64.b64decode(left_current["image"]), np.uint8
                            #     ),
                            #     cv2.IMREAD_COLOR,
                            # )
                            current_line = next(left_iterator, None)  # Read next line
                            if current_line and current_line.strip():
                                left_current = json.loads(current_line.strip())
                            else:
                                left_current = None  # End of file or empty line

                    # Process right camera
                    if right_iterator and right_current:
                        while (
                            right_current
                            and CommonUtil.timestamp_to_nanosec(
                                right_current["timestamp"]
                            )
                            <= timestamp_nanoseconds
                        ):
                            # right_image = cv2.imdecode(
                            #     np.frombuffer(
                            #         base64.b64decode(right_current["image"]), np.uint8
                            #     ),
                            #     cv2.IMREAD_COLOR,
                            # )
                            current_line = next(right_iterator, None)  # Read next line
                            if current_line and current_line.strip():
                                right_current = json.loads(current_line.strip())
                            else:
                                right_current = None  # End of file or empty line

                    # Decode the images
                    left_image_n = decode_image(left_current)
                    right_image_n = decode_image(right_current)

                    # Update only if new image is available
                    if left_image_n is not None:
                        left_image = left_image_n
                    if right_image_n is not None:
                        right_image = right_image_n

                    # Concatenate images vertically (right, top, left)
                    combined_image = cv2.vconcat([right_image, top_image, left_image])

                    # Save frame as JPEG
                    frame_filename = os.path.join(
                        temp_frames_folder, f"frame_{frame_count:09d}.jpg"
                    )
                    cv2.imwrite(frame_filename, combined_image)
                    frame_count += 1

        # Close the iterators
        if left_iterator:
            left_iterator.close()
        if right_iterator:
            right_iterator.close()

        # Release the video writer (no longer used)
        # video_writer.release()

        if frame_count > 0:
            # Use ffmpeg to create video from JPEG frames
            input_pattern = os.path.join(temp_frames_folder, "frame_%09d.jpg")

            commands = [
                # VA-API
                f"ffmpeg -y -framerate {fps} -i {input_pattern} "
                "-vaapi_device /dev/dri/renderD128 -vf format=nv12,hwupload -c:v h264_vaapi -qp 25 "
                f"{output_video_file}",
                # Fallback - properly handle color range to avoid deprecated pixel format warning
                f"ffmpeg -y -framerate {fps} -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video_file}",
            ]

            # Ignore VA-API version if CPU-only is requested
            if cpu_only:
                commands = [commands[-1]]

            for c in commands:
                try:
                    result = SystemUtil.run_command(c, check=True, capture_output=True)

                    if result.returncode == 0:
                        logger.info(
                            cls.__name__, f"video generated at {output_video_file}"
                        )
                        break
                except subprocess.CalledProcessError as e:
                    logger.error(
                        cls.__name__,
                        f"Error during FFmpeg processing: {e}",
                        f"stdout: {e.stdout}",
                        f"stderr: {e.stderr}",
                    )
        else:
            print("No frames were processed to generate video.")

        # Clean up temporary frames folder
        if os.path.exists(temp_frames_folder):
            shutil.rmtree(temp_frames_folder)


def main():

    base_path = str(
        pathlib.Path("/home/we/Projects/miniROS/tmp_data/126cc7a4-1996-4b96-b70c-3b6a7165c484").resolve()
    )

    # Run VA-API variant first
    start_time = TimeUtil.now()
    VideoUtil.generate_video(base_path, cpu_only=False)

    vaapi_ms = TimeUtil.get_elapsed_time_ms(start_time)

    logger.info(f"VA-API: {vaapi_ms:.2f} ms")

    # Run CPU variant
    start_time = TimeUtil.now()
    VideoUtil.generate_video(base_path, cpu_only=True)

    cpu_ms = TimeUtil.get_elapsed_time_ms(start_time)

    logger.info(f"CPU: {cpu_ms:.2f} ms")

    speedup = 1 / (vaapi_ms / cpu_ms)

    logger.info(f"VA-API: {vaapi_ms:.2f} ms, CPU: {cpu_ms:.2f} ms")
    logger.info(f"Speedup: {speedup:.2f}x")


if __name__ == "__main__":
    main()
