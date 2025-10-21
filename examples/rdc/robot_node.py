"""
Robot node.
- ALIGN: Aligning robot to gello's position. (SLOW)
- ENABLE: Enable the moving of robot. (The FilIO is off).
- RECORD: Recording state. (The FilIO is on).
- STOP: Pause the robot (Can interrupt anytime).
- DRAG: Enter the drag mode, when you can drag the robot.
"""
import asyncio
# from .ports import ROBOT_OBS_PORT, RDC_STATE_PUB_PORT, RDC_STATE_RECV_PORT
from typing import Dict, Any
from mini_ros.devices.robots.marvin_robot import MarvinRobot
from mini_ros.devices.robots.pika_gripper import PikaGripper
from mini_ros.common.state import RDCState, RobotNodeState
from mini_ros.common.device import Robot
from mini_ros.network.network_queue import QueueDealerSender, QueueSubRecver



class RobotNode:
    """
    Robot node.
    """
    def __init__(self, robots: Dict[str, Robot], node_cfg: Dict[str, Any] = {}):
        self.robots = robots
        # self.state: RDCState = RDCState.INIT
        # Network interface
        # Send robot observations to the FileIONode
        self.obs_sender = QueueDealerSender(name="robot_obs", port=node_cfg.get("obs_port", 5005))
        # Receive the RDC state from the RDC state node
        self.rdc_state_recver = QueueSubRecver(name="rdc_state", port=node_cfg.get("rdc_state_pub_port", 5006))
        # Send internal state to the RDC state node
        self.robot_node_state_sender = QueueDealerSender(name="robot_node_state", port=node_cfg.get("rdc_state_recv_port", 5007))
        # Internal buffer for sharing control & obs
        self._obs_buffer = asyncio.Queue(maxsize=1)
        self._control_buffer = asyncio.Queue(maxsize=1)
        # Mutex
        self.inter_state_mutex = asyncio.Lock()
        self.rdc_state = RDCState.INIT
        self.inter_state = RobotNodeState.INIT

    async def control_loop(self):
        pass

    async def obs_loop(self):
        pass