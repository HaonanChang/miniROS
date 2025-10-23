"""
Saving utilities.
Handle compression and serialization of data & deserialization.
"""
import enum
import queue
import pickle
from mini_ros.common.state import RobotAction, RobotState
from typing import Dict, Any, Union
import threading
from loguru import logger


class RecordStyle(enum.Enum):
    """
    Style of data to save.
    """
    WORLD_ENGINE_RDC = 0


class RecorderConfig:
    save_dir: str
    save_style: RecordStyle
    buffer_size: int = 72000  # (20 mins for 60 fps)


class EpisodeRecorder:
    """
    Episode recorder.
    Caches the data for a whole episode, and save it to the given path after the episode ends.
    Handle compression and serialization of data & deserialization.
    """
    def __init__(self, save_style: RecordStyle):
        self.save_style = save_style
        self.data_buffers: Dict[str, queue.Queue[Any]] = {
        }
        self.io_mutex = threading.Lock()
        self.is_active_event = threading.Event()
        self.data_folder = None

    def switch_data_folder(self, data_folder: str):
        """Select the dumping place"""
        if self.is_active():
            logger.warning("Recorder is active, can't switch data folder")
            return
        with self.io_mutex:
            self.data_folder = data_folder

    def is_active(self) -> bool:
        """
        Active status means the recorder can receive data.
        """
        return self.is_active_event.is_set()

    def start(self):
        """
        Start the recorder.
        """        
        if self.is_active():
            logger.warning("Recorder is already active, can't start again")
            return
        # Clear the data buffers
        with self.io_mutex:
            self.data_buffers = {key: queue.Queue(maxsize=self.buffer_size) for key in self.data_buffers.keys()}
        self.is_active_event.set()

    def stop(self):
        self.is_active_event.clear()

    def put(self, data: Any, key: str):
        """
        Put the data into the data buffer.
        """
        if not self.is_active():
            logger.warning("Recorder is not active, can't put data")
            return
        with self.io_mutex:
            if key not in self.data_buffers:
                self.data_buffers[key] = queue.Queue(maxsize=self.buffer_size)
            self.data_buffers[key].put(data)

    def save(self) -> Any:
        """
        Save the data to the given path.
        Saving is blocking. It will set the active event to False.
        And occupy the io_mutex.
        """
        self.is_active_event.clear()
        with self.io_mutex:
            if self.save_style == RecordStyle.WORLD_ENGINE_RDC:
                # World Engine RDC style:
                # Save the data into pickle files.
                for key, buffer in self.data_buffers.items():
                    if buffer.empty():
                        continue
                    data_list = []
                    while not buffer.empty():
                        data = buffer.get()
                        if isinstance(data, RobotAction):
                            serialized_data = self.compress(data, self.save_style)
                            data_list.append(serialized_data)
                    with open(f"{self.data_folder}/{key}.pkl", "wb") as f:
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
                    "commander_state": data.data_status,
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
                    "commander_state": data.data_status,
                }
            else:
                raise ValueError(f"Invalid data type: {type(data)}")
        else:
            raise ValueError(f"Invalid record style: {record_style}")