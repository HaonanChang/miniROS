"""
Saving utilities.
Handle compression and serialization of data & deserialization.
"""
import enum
import queue
from dataclasses import dataclass
import pickle
from mini_ros.common.device import Recorder
from mini_ros.common.state import RobotAction, RobotState
from typing import Dict, Any, Union
import threading
from loguru import logger
import os


class RecordStyle(enum.Enum):
    """
    Style of data to save.
    """
    WORLD_ENGINE_RDC = 0


@dataclass
class RecorderConfig:
    name: str
    fps: int
    max_episode_length: int  # in seconds
    data_root_dir: str
    save_at_stop: bool = True
    record_style: RecordStyle = RecordStyle.WORLD_ENGINE_RDC


class EpisodeRecorder(Recorder):
    """
    Episode recorder.
    Caches the data for a whole episode, and save it to the given path after the episode ends.
    Handle compression and serialization of data & deserialization.
    """
    def __init__(self, config: RecorderConfig):
        self.name = config.name
        self.record_style = config.record_style
        self.buffer_size = config.fps * config.max_episode_length
        self.data_root_dir = config.data_root_dir
        self.episode_name = ""
        self.data_buffers: Dict[str, queue.Queue[Any]] = {
        }
        self.io_mutex = threading.Lock()
        self.is_active_event = threading.Event()
        self.data_folder = None
        self.save_at_stop = config.save_at_stop

    def initialize(self):
        pass

    def is_active(self) -> bool:
        """
        Active status means the recorder can receive data.
        """
        return self.is_active_event.is_set()

    def start(self):
        pass

    def stop(self):
        pass

    def pause(self):
        pass

    def start_record(self, episode_name: str):
        """
        NOTE: Start is a blocking call.
        """       
        if self.is_active():
            logger.warning("Recorder is already active, can't start again. Close it first.")
            return
        logger.info(f"Starting recorder: {self.name}")
        # Clear the data buffers
        with self.io_mutex:
            self.data_buffers = {key: queue.Queue(maxsize=self.buffer_size) for key in self.data_buffers.keys()}
            self.episode_name = episode_name
        self.is_active_event.set()

    def stop_record(self):
        """
        NOTE: Stop recording is a blocking call.
        """
        logger.info(f"Stopping recorder: {self.name}")
        self.is_active_event.clear()
        # Call save method
        if self.save_at_stop:
            self.save()

    def put(self, data: Any, key: str):
        """
        Put the data into the data buffer.
        NOTE: Put must be a non-blocking call. 
        Because it will be called in other time-sensitive threads.
        """
        if not self.is_active():
            # logger.warning(f"Recorder {self.name} is not active, can't put data")
            return
        else:
            # logger.info(f"Putting data to recorder: {self.name}, key: {key}")
            pass
        # with self.io_mutex:
        if key not in self.data_buffers:
            self.data_buffers[key] = queue.Queue(maxsize=self.buffer_size)
        self.data_buffers[key].put_nowait(data)

    def save(self) -> Any:
        """
        Save the data to the given path.
        Saving is blocking. It will set the active event to False.
        And occupy the io_mutex.
        NOTE: Save is a blocking call.
        """
        logger.info(f"Saving recorder: {self.name}, keys: {self.data_buffers.keys()}")
        self.is_active_event.clear()
        with self.io_mutex:
            if self.record_style == RecordStyle.WORLD_ENGINE_RDC:
                # World Engine RDC style:
                # Save the data into pickle files.
                for key, buffer in self.data_buffers.items():
                    if buffer.empty():
                        continue
                    data_list = []
                    while not buffer.empty():
                        data = buffer.get()
                        serialized_data = self.compress(data, self.record_style)
                        data_list.append(serialized_data)
                    data_folder = f"{self.data_root_dir}/{self.episode_name}"
                    os.makedirs(data_folder, exist_ok=True) 
                    logger.info(f"Saving data to {data_folder}/{self.name}_{key}.pkl")
                    with open(f"{data_folder}/{self.name}_{key}.pkl", "wb") as f:
                        pickle.dump(data_list, f)

    @classmethod
    def compress(cls, data: Union[RobotAction, RobotState], record_style: RecordStyle) -> bytes:
        """
        Compress the data.
        Different record styles may have different compression methods.
        """
        if record_style == RecordStyle.WORLD_ENGINE_RDC:
            if isinstance(data, RobotAction):
                return {
                    "action_timestamp": data.timestamp,
                    "action_gello_timestamp": data.timestamp_recv,
                    "action": data.joint_cmds,
                    "commander_state": data.code,
                }
            elif isinstance(data, RobotState):
                return {
                    "robot_timestamp": data.timestamp,
                    "joint_positions": data.joint_positions,
                    "joint_velocities": data.joint_velocities,
                    "currents": data.joint_currents,
                    "joint_currents": data.joint_currents,
                    "torques": data.joint_efforts,
                    "ee_pos": data.end_effector_positions,
                    "commander_state": data.code,
                }
            else:
                raise ValueError(f"Invalid data type: {type(data)}")
        else:
            raise ValueError(f"Invalid record style: {record_style}")