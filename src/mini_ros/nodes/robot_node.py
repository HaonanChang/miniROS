"""
Robot node.
- ALIGN: Aligning robot to gello's position. (SLOW)
- ENABLE: Enable the moving of robot. (The FilIO is off).
- RECORD: Recording state. (The FilIO is on).
- STOP: Pause the robot (Can interrupt anytime).
- DRAG: Enter the drag mode, when you can drag the robot.
"""
import asyncio
from loguru import logger
# from .ports import ROBOT_OBS_PORT, RDC_STATE_PUB_PORT, RDC_STATE_RECV_PORT
from typing import Dict, Any
from mini_ros.devices.robots.marvin_robot import MarvinRobot
from mini_ros.devices.robots.pika_gripper import PikaGripper
from mini_ros.common.state import CommanderState, RobotNodeState
from mini_ros.common.device import Robot, Device
from mini_ros.network.network_queue import QueueDealerSender, QueueSubRecver
from mini_ros.utils.rate_limiter import RateLimiterAsync
from mini_ros.utils.async_util import AsyncUtil



class RobotNode:
    """
    Robot node. 
    Notice that Currently all robot has to be Full-duplex.
    As in this framework, the read & write can happend at the same time.
    TODO: Support Half-duplex.
    """
    def __init__(self, devices: Dict[str, Robot | Device], node_cfg: Dict[str, Any] = {}):
        self.devices = devices
        # self.state: RDCState = RDCState.INIT
        # Network interface
        name_prefix = node_cfg.get("name_prefix", "robot")
        name_suffix = node_cfg.get("name_suffix", "")
        name = f"{name_prefix}{name_suffix}"
        # Send robot observations to the FileIONode
        self.obs_sender = QueueDealerSender(name=f"{name}_obs", port=node_cfg.get("obs_port", 5005))
        # Receive the RDC state from the RDC state node
        self.commander_state_recver = QueueSubRecver(name=f"{name}_commander_state", port=node_cfg.get("commander_state_pub_port", 5008))
        # Send internal state to the RDC state node
        self.robot_node_state_sender = QueueDealerSender(name=f"{name}_state", port=node_cfg.get("device_state_recv_port", 5007))
        # Internal buffer for sharing control & obs
        self._obs_buffer: Dict[str, asyncio.Queue] = {name: asyncio.Queue(maxsize=1) for name in devices.keys()}
        self._control_buffer = Dict[str, asyncio.Queue] = {name: asyncio.Queue(maxsize=1) for name in devices.keys()}
        # Mutex
        self.inter_state_mutex = asyncio.Lock()
        self.commander_state = CommanderState.INIT
        self.inter_state = RobotNodeState.INIT
        # Rate limiters
        self._commander_listen_rate_limiter = RateLimiterAsync(rate=node_cfg.get("commander_listen_rate", 50))
        self._listen_rate_limiters = {name: RateLimiterAsync(rate=node_cfg.get(f"{name}_listen_rate", 200)) for name in devices.keys() if isinstance(devices[name], Reader) or isinstance(devices[name], Robot)}
        self._control_rate_limiters = {name: RateLimiterAsync(rate=node_cfg.get(f"{name}_control_rate", 60)) for name in devices.keys() if isinstance(devices[name], Robot)}
    
    async def initialize(self):
        # Initialize all devices
        for name in self.devices.keys():
            self.devices[name].initialize()

    async def start(self):
        try:
            await asyncio.gather(
                self.commander_listen_loop(),
                *[self.listen_loop(name) for name in self.devices.keys()],
                *[self.control_loop(name) for name in self.devices.keys()],
            )
        except KeyboardInterrupt:
            logger.info("RobotNode received keyboard interrupt! (Ctrl+C?)")
        except asyncio.CancelledError:
            logger.info("RobotNode received cancellation signal from asyncio!")
        except Exception as e:
            logger.error(f"Error in robot node: {e}")
        finally:
            self.stop()

    async def commander_listen_loop(self):
        while True:
            await self._commander_listen_rate_limiter.wait_for_tick("RobotNode_commander_listen_task")
            recv_commander_state = await self.commander_state_recver.get()
            async with self.inter_state_mutex:
                if recv_commander_state is not None:
                    self.commander_state = recv_commander_state
                else:
                    self.commander_state = CommanderState.LOSE_CONNECTION
            await self._commander_listen_rate_limiter.unset_busy("RobotNode_commander_listen_task")
    
    async def listen_loop(self, name: str):
        if isinstance(self.devices[name], Robot) and not self.devices[name].is_full_duplex:
            logger.warning(f"Device {name} is not a full-duplex robot, skipping commander normal listen loop!")
            return
        while True:
            await self._listen_rate_limiters[name].wait_for_tick(f"RobotNode_listen_task_{name}")
            device_state = await self.devices[name].get_state()
            await self._obs_buffer[name].put(device_state)
            await self._listen_rate_limiters[name].unset_busy(f"RobotNode_listen_task_{name}")

    async def control_loop(self, name: str):
        if isinstance(self.devices[name], Robot) and not self.devices[name].is_full_duplex:
            logger.warning(f"Device {name} is not a full-duplex robot, skipping normal control loop!")
            return
        if not isinstance(self.devices[name], Robot):
            logger.warning(f"Device {name} is not a robot, skipping control loop!")
            return
        while True:
            await self._control_rate_limiters[name].wait_for_tick(f"RobotNode_control_task_{name}")
            # Change strategy based on robot state
            async with self.inter_state_mutex:
                inter_state = self.inter_state
            
            if inter_state == RobotNodeState.CONTROL:
                # In-control, use control buffer to control the robot
                control_data = await self._control_buffer[name].get()
                if control_data is not None:
                    await self.devices[name].apply_action(control_data)
                else:
                    logger.warning(f"Control data for {name} is None!")
            elif inter_state == RobotNodeState.PAUSE:
                # In-stop, stop the robot
                await AsyncUtil.run_blocking_as_async(self.devices[name].pause)
            elif inter_state == RobotNodeState.DRAG:
                # In-drag, set the robot to drag mode
                await AsyncUtil.run_blocking_as_async(self.devices[name].switch_mode, "drag")
            await self._control_rate_limiters[name].unset_busy(f"RobotNode_control_task_{name}")

    def stop(self):
        for name in self.devices.keys():
            self.devices[name].stop()