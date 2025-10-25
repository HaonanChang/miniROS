"""
Commander node.
"""
# import asyncio
import copy
import numpy as np
import threading
from typing import Dict, Any, Awaitable, Callable, Type, List
from dataclasses import dataclass, field
from mini_ros.common.state import ConsoleState, RobotState
from mini_ros.common.device import Device
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.rate_limiter import RateLimiterSync
from mini_ros.network.network_queue import One2ManySender, Many2OneRecver
from loguru import logger


@dataclass
class ProxyFunc:
    func: Callable
    args: Dict[str, Type] = field(default_factory=dict)


@dataclass
class GelloConsoleConfig:
    """
    Configuration for the GelloConsole device.
    """
    # From gello name to the list of joints.
    # Example:
    # gello_mapping: {
    #     "left": {"gello", [0, 1, 2, 3, 4, 5, 6]},
    #     "right": {"gello", [7, 8, 9, 10, 11, 12, 13]},
    # }
    gello_mapping: Dict[str, Any] = field(default_factory=dict)
    goal_gello_states: Dict[str, Any] = field(default_factory=dict)
    # Example:
    # goal_gello_states: {
    #     "gello": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}
    gello_goal_tolerance: Dict[str, Any] = field(default_factory=dict)
    # Example:
    # gello_goal_tolerance: {
    #     "gello": [0.3] * 14,  
    # }
    # Read rate limit.
    read_rate: Dict[str, int] = field(default_factory=dict)
    # Proceed rate limit.
    proceed_rate: int = 50  # Hz


class GelloConsole(Device):
    """
    GelloConsole device. Used for control teleop-ed robot.
    """

    def __init__(self, gellos: List[Device], config: GelloConsoleConfig):
        self.console_stage = ConsoleState.INIT
        self.gellos: Dict[str, Device] = {gello.name: gello for gello in gellos}
        self.gello_states: Dict[str, RobotState] = {gello_name: RobotState() for gello_name in self.gellos.keys()}
        self._console_mutex: threading.Lock = threading.Lock()
        self._gello_mutex: Dict[str, threading.Lock] = {gello_name: threading.Lock() for gello_name in self.gellos.keys()}
        # Paddle-related annoyances -- Sometimes browser / operator double-fires paddle presses that confuses the robot
        self._paddle_mutex: threading.Lock = threading.Lock()
        self._is_paddle_busy: bool = False
        # Rate limiters
        self.proceed_rate = config.proceed_rate
        self.gello_mapping: Dict[str, Any] = config.gello_mapping
        self.goal_gello_states: Dict[str, Any] = config.goal_gello_states
        self.gello_goal_tolerance: Dict[str, Any] = config.gello_goal_tolerance
        self.stop_event = threading.Event()

    def is_alive(self) -> bool:
        return not self.stop_event.is_set()
        
    def initialize(self):
        pass

    def start(self):
        self.proceed_thread = threading.Thread(target=self.proceed_loop, daemon=True)
        self.proceed_thread.start()

    def read_loop(self, gello_name: str):
        read_rate_limiter = RateLimiterSync(self.read_rate[gello_name])
        while True:
            if not self.is_alive():
                break
            try:
                read_rate_limiter.wait_for_tick()
                with self._gello_mutex[gello_name]:
                    self.gello_states[gello_name] = self.gellos[gello_name].get_state()
            except Exception as e:
                logger.error(f"Error in GelloConsole: {e}")
                break

    def proceed_loop(self):
        proceed_rate_limiter = RateLimiterSync(self.proceed_rate)  # Proceed in 50 Hz
        try:
            while True:
                if not self.is_alive():
                    break
                try:
                    # Manage the state machine.
                    proceed_rate_limiter.wait_for_tick()
                    self.update_state()
                except Exception as e:
                    logger.error(f"Error in WebConsole: {e}")
                    break
            logger.info("WebConsole stopped!")
        except Exception as e:
            logger.error(f"Error in WebConsole: {e}")
            self.stop()
    
    def stop(self):
        self.stop_event.set()
        self.proceed_thread.join()

    def update_state(self):
        with self._console_mutex:
            state = self.console_stage
        if state == ConsoleState.ALIGN:
            # Wait for the robot to be aligned.
            if not self._is_robot_gello_aligned():
                TimeUtil.sleep_by_ms(10)
            else:
                # Move to the next state.
                self.console_stage = ConsoleState.ACTIVE

    def get_states(self) -> Dict[str, Any]:
        with self._console_mutex:
            console_state = self.console_stage
        with self._gello_mutex:
            gello_states = copy.deepcopy(self.gello_states)
        gello_deltas = { }
        gello_aligned = { }
        for gello_name, gello_state in gello_states.items():
            gello_deltas[gello_name] = np.array(gello_state.joint_positions) - np.array(self.goal_gello_states[gello_name])
            gello_aligned[gello_name] = np.all(np.abs(
                np.array(gello_state.joint_positions) - np.array(self.goal_gello_states[gello_name])) < np.array(self.gello_goal_tolerance[gello_name]))
       
        # Refactor the gello states to the frontend format.
        visual_gello_states = {gello_name: {"joints": [], "aligned": [], "deltas": []} for gello_name in self.gello_mapping.keys()}
        for gello_name, gello_state in gello_states.items():
            for gello_tuple in self.gello_mapping[gello_name]:
                gello_name, joints = gello_tuple
                visual_gello_states[gello_name]["joints"] += gello_state.joint_positions[joints].tolist()
                visual_gello_states[gello_name]["aligned"] += gello_aligned[gello_name][joints].tolist()
                visual_gello_states[gello_name]["deltas"] += gello_deltas[gello_name][joints].tolist()
        
        is_gello_home = np.all(gello_aligned.values())
        
        return {
            "commander_state": console_state,
            "gello_state": visual_gello_states,
            "counter": 0,
            "is_gello_home": is_gello_home,
            "is_connection_bad": False,
            "last_server_comm_time": 0,
            "verson": 0, 
            "is_autopilot": False,
        }
    
    def get_health_state(self) -> Dict[str, Any]:
        with self._console_mutex:
            commander_state = self.console_stage
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
    def make_paddle_handler(self, action_func: Callable[[], None]):
        """
        Make a paddle handler that prevents race conditions and mis-fires during reset.
        Args:
            action_func: The function to call when the paddle is pressed.
        Returns:
            A function that can be used as a paddle handler.
        """

        def _handler():

            with self._paddle_mutex:
                if self._is_paddle_busy:
                    raise Exception("Cannot perform action while resetting!")

                action_func()

        return _handler

    def left_paddle_handler(self):
        """
        Functions when left paddle / "s" is pressed.
        Left paddle handles all conception with startting.
        """
        logger.info("Left paddle pressed!")
        with self._console_mutex:
            if self.console_stage == ConsoleState.INIT:
                self.console_stage = ConsoleState.ALIGN
            elif self.console_stage == ConsoleState.ACTIVE:
                self.console_stage = ConsoleState.RECORD
            elif self.console_stage == ConsoleState.RECORD:
                self.console_stage = ConsoleState.ACTIVE
                # Finish one recording. # Mark the current episode as failed.
            else:
                raise Exception("Invalid commander state!")

    def middle_paddle_handler(self):
        """
        Functions when middle paddle / "d" is pressed.
        Middle paddle handles all conception with switching.
        """
        logger.info("Middle paddle pressed!")
        with self._console_mutex:
            if self.console_stage == ConsoleState.RECORD:
                self.console_stage = ConsoleState.ACTIVE
                # Finish one recording. # Mark the current episode as successful.
            else:
                raise Exception("Invalid commander state!")

    def right_paddle_handler(self):
        """
        Functions when right paddle / "q" is pressed.
        Right paddle handles all conception with stopping.
        """
        logger.info("Right paddle pressed!")
        with self._console_mutex:
            if not self.console_stage == ConsoleState.STOPPED:
                self.console_stage = ConsoleState.STOPPED
            else:
                self.console_stage = ConsoleState.INIT

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

    #########################################################
    # Helper functions
    #########################################################
    def _is_robot_gello_aligned(self) -> bool:
        """
        Check if the robot is aligned to the gello.

        """
        return True