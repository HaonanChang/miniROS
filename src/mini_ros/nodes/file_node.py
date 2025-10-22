"""
File node, handling IO heavy tasks. 
In current version, cameras nodes are under this node.
- ON: On the file node.
- SAVE: Save the data to the file.
- DROP: Drop the data to the file.
- OFF: Off the file node.
"""
import asyncio
from typing import Dict, Any
from mini_ros.common.state import RDCState
from mini_ros.network.network_queue import QueueDealerSender, QueueSubRecver
from mini_ros.common.device import Camera


class FileNode:
    """
    File node.
    """
    def __init__(self, cameras: Dict[str, Camera], file_cfg: Dict[str, Any] = {}):
        self.file_cfg = file_cfg
        self.rdc_state_recver = QueueSubRecver(name="rdc_state", port=file_cfg.get("rdc_state_pub_port", 5006))
        self.file_node_state_sender = QueueDealerSender(name="file_node_state", port=file_cfg.get("rdc_state_recv_port", 5007))

    async def camera_obs_loop(self):
        pass

    async def file_write_loop(self):
        pass