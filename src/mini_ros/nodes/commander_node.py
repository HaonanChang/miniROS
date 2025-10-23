"""
State node.
"""
import asyncio
from typing import Dict, Any, Awaitable, Callable, Type
from dataclasses import dataclass, field
from mini_ros.common.state import CommanderState
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.rate_limiter import RateLimiterAsync
from mini_ros.network.network_queue import QueuePubSender, QueueRouterRecver
from loguru import logger


@dataclass
class ProxyFunc:
    func: Callable
    args: Dict[str, Type] = field(default_factory=dict)


class CommanderNode:
    """
    Commander node.
    """
    def __init__(self, commander_cfg: Dict[str, Any] = {}):
        self.commander_cfg = commander_cfg
        self.commander_state = CommanderState.INIT
        self._state_mutex: asyncio.Lock = asyncio.Lock()  # State guarding
        # Paddle-related annoyances -- Sometimes browser / operator double-fires paddle presses that confuses the robot
        self._reset_mutex: asyncio.Lock = asyncio.Lock()
        self._is_reset_busy: bool = False
        self._paddle_mutex: asyncio.Lock = asyncio.Lock()
        # Rate limiters
        self._process_rate_limiter = RateLimiterAsync(50)  # Refresh SM in 50 Hz
        # Network interface
        # Publish commander state to all devices
        self.commander_state_sender = QueuePubSender(name="commander_state", port=commander_cfg.get("commander_state_pub_port", 5008))
        # Receive device state from all devices
        self.device_state_recver = QueueRouterRecver(name="device_state", port=commander_cfg.get("device_state_recv_port", 5009))

    async def initialize(self):
        pass

    async def start(self):
        try:
            while True:
                await self.proceed()
        except KeyboardInterrupt:
            logger.info("Commander received keyboard interrupt! (Ctrl+C?)")
        except asyncio.CancelledError:
            logger.info("Commander received cancellation signal from asyncio!")
        except Exception as e:
            logger.error(f"Error in commander: {e}")
        finally:
            self.stop()

    async def proceed(self):
        # Manage the state machine.
        await self._process_rate_limiter.wait_for_tick("CommanderNode_proceed")
        async with self._state_mutex:
            state = self.commander_state
        if state == CommanderState.ALIGN:
            # Wait for the robot to be aligned.
            if not await self._is_robot_gello_aligned():
                await TimeUtil.sleep_by_ms(10)
            else:
                # Move to the next state.
                self.commander_state = CommanderState.ACTIVE
        await self._process_rate_limiter.unset_busy("CommanderNode_proceed")

    async def get_states(self) -> Dict[str, Any]:
        async with self._state_mutex:
            commander_state = self.commander_state
        return {
            "commander_state": commander_state,
            "gello_state": {
                "left": {
                    "joints": [0, 0, 0, 0, 0, 0, 0],
                    "aligned": [True, True, True, True, True, True, True],
                    "deltas": [0, 0, 0, 0, 0, 0, 0],
                },
                "right": {
                    "joints": [0, 0, 0, 0, 0, 0, 0],
                    "aligned": [True, True, True, True, True, True, True],
                    "deltas": [0, 0, 0, 0, 0, 0, 0],
                },
            },
            "counter": 0,
            "is_gello_home": True,
            "is_connection_bad": False,
            "last_server_comm_time": 0,
            "verson": 0, 
            "is_autopilot": False,
        }
    
    async def get_health_state(self) -> Dict[str, Any]:
        async with self._state_mutex:
            commander_state = self.commander_state
        return {
            "commander_state": commander_state,
            "counter": 0,
            "is_gello_home": True,
                "is_connection_bad": False,
                "verson": 0, 
                "is_autopilot": False,
            } 
    
    def reset_counter(self):
        pass

    #########################################################
    # Paddle related: Providing control interface
    #########################################################
    def make_paddle_handler(self, action_func: Callable[[], Awaitable[None]]):
        """
        Make a paddle handler that prevents race conditions and mis-fires during reset.
        Args:
            action_func: The function to call when the paddle is pressed.
        Returns:
            A function that can be used as a paddle handler.
        """

        async def _handler():

            async with self._paddle_mutex:
                if self._is_reset_busy:
                    raise Exception("Cannot perform action while resetting!")

                await action_func()

        return _handler

    async def left_paddle_handler(self):
        """
        Functions when left paddle / "s" is pressed.
        Left paddle handles all conception with startting.
        """
        logger.info("Left paddle pressed!")
        async with self._state_mutex:
            if self.commander_state == CommanderState.INIT:
                self.commander_state = CommanderState.ALIGN
            elif self.commander_state == CommanderState.ACTIVE:
                self.commander_state = CommanderState.RECORD
            elif self.commander_state == CommanderState.RECORD:
                self.commander_state = CommanderState.ACTIVE
                # Finish one recording. # Mark the current episode as failed.
            else:
                raise Exception("Invalid commander state!")

    async def middle_paddle_handler(self):
        """
        Functions when middle paddle / "d" is pressed.
        Middle paddle handles all conception with switching.
        """
        logger.info("Middle paddle pressed!")
        async with self._state_mutex:
            if self.commander_state == CommanderState.RECORD:
                self.commander_state = CommanderState.ACTIVE
                # Finish one recording. # Mark the current episode as successful.
            else:
                raise Exception("Invalid commander state!")

    async def right_paddle_handler(self):
        """
        Functions when right paddle / "q" is pressed.
        Right paddle handles all conception with stopping.
        """
        logger.info("Right paddle pressed!")
        async with self._state_mutex:
            if not self.commander_state == CommanderState.STOPPED:
                self.commander_state = CommanderState.STOPPED
            else:
                self.commander_state = CommanderState.INIT

    def get_api_mapping(self) -> Dict[str, Any]:
        """
        Expose the API to the Frontend.
        """
        return {
            # New APIs for Paddle control.
            "left_paddle": ProxyFunc(
                self.make_paddle_handler(self.left_paddle_handler)
            ),
            "middle_paddle": ProxyFunc(self.make_paddle_handler(self.middle_paddle_handler)),
            "right_paddle": ProxyFunc(self.make_paddle_handler(self.right_paddle_handler)),
            "get_states": ProxyFunc(self.get_states),
            "get_health_state": ProxyFunc(self.get_health_state),
        }

    def stop(self):
        pass

    #########################################################
    # Helper functions
    #########################################################
    async def _is_robot_gello_aligned(self) -> bool:
        """
        Check if the robot is aligned to the gello.

        """
        return True