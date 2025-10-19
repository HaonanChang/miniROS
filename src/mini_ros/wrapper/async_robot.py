"""
Async wrapper for robot.

There is one long-running task for reading the state of the robot.
"""
import asyncio
from loguru import logger
from typing import Any, Callable, Coroutine, Dict, Set
from mini_ros.common.error import ReadError
from mini_ros.utils.rate_limiter import RateLimiter
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.state import RobotDeviceState, RobotState, RobotAction, TimedData
from mini_ros.common.device import Device, Robot

DEBUG_ASYNC_INPUT = True


class AsyncRobot(Device):
    """
    AsyncIO Wrapper for Robot. Support multiple robots.
    State:
        INIT -> OPEN -> RUNNING -> STOPPED
    
    NOTICE: There is one thing I need to think, should I have a reading limiter?
    """

    def __init__(self, robots: Dict[str, Robot], robot_configs: Dict[str, Any], polling_rate: Dict[str, int], writing_rate: Dict[str, int], timeout: float = 1.0):
        """
        Initialize the InputDeviceAsyncWrapper object
        """

        super().__init__(f"{'&'.join(robots.keys())}")
        assert len(robots) == len(robot_configs), "Number of drivers and driver configs must be the same"
        self.robots: Dict[str, Robot] = robots
        self.robot_configs: Dict[str, Any] = robot_configs
        self.read_limiters: Dict[str, RateLimiter] = {driver_name: RateLimiter(polling_rate[driver_name]) for driver_name in robots.keys()}
        self.write_limiters: Dict[str, RateLimiter] = {driver_name: RateLimiter(writing_rate[driver_name]) for driver_name in robots.keys()}
        self._timeout: float = timeout
        # Internal state
        self.state: RobotDeviceState = RobotDeviceState.INIT
        self.condition: asyncio.Condition = asyncio.Condition()

        # Background task for the reading loop
        self._polling_read_tasks: Dict[str, asyncio.Task] = {}

        # Store the input value for each driver, coroutine safe
        self._input_queues: Dict[str, asyncio.Queue] = {driver_name: asyncio.Queue(maxsize=1) for driver_name in robots.keys()}
        self._event_listeners: Dict[str, Set[Callable[..., Coroutine[Any, Any, Any]]]] = {driver_name: set([]) for driver_name in robots.keys()}

    async def wait_for_state(self, target_state: RobotDeviceState):
        async with self.condition:
            await self.condition.wait_for(lambda: self.state == target_state)

    async def set_state(self, new_state: RobotDeviceState):
        async with self.condition:
            self.state = new_state
            self.condition.notify_all()

    async def initialize(self):
        """
        Perform handshake with the device and establish communication
        """

        for driver_name, reader_config in self.robot_configs.items():
            await AsyncUtil.run_blocking_as_async(self.robots[driver_name].initialize, reader_config)
        await self.set_state(RobotDeviceState.OPEN)

    async def start(self) -> None:
        """
        Start the polling loop
        (Call this method before making any get_state() calls
        or else expect lags from the first get_state() call)
        """

        await self.wait_for_state(RobotDeviceState.OPEN)
        self._polling_read_tasks = {driver_name: AsyncUtil.detach_coroutine(self._reading_loop(driver_name)) for driver_name in self.robots.keys()}
        for driver_name in self.robots.keys():
            self._polling_read_tasks[driver_name].add_done_callback(lambda _: self._input_queues[driver_name].task_done())
        await TimeUtil.sleep_by_ms(500)  # Wait for the reading loop to start
        await self.set_state(RobotDeviceState.RUNNING)
        
    async def add_listener(
        self, driver_name: str, listener: Callable[..., Coroutine[Any, Any, Any]]
    ) -> None:
        """
        Add a listener to the PollingInput for a specific driver.
        """

        self._event_listeners[driver_name].add(listener)

    async def get_state(self, driver_name: str) -> RobotState:
        """
        Reading from the device. Limit reading rate by the driver rate.

        Can only be called when the input device is in RUNNING state.

        Returns:
            Any (You should know and override the type of this return value): The input value from the Polling device
        """

        await self.wait_for_state(RobotDeviceState.RUNNING)

        # Handover wait for the value to be available to the reading loop
        val = await self._input_queues[driver_name].get()  # Block until a value is available

        return val

    async def apply_action(self, driver_name: str, action: RobotAction) -> RobotAction:
        """
        Apply action to the robot
        """
        await self.wait_for_state(RobotDeviceState.RUNNING)

        limiter = self.write_limiters[driver_name]
        await limiter.wait_for_tick("AsyncRobot_apply_action")

        action = await AsyncUtil.run_blocking_as_async(self.robots[driver_name].apply_action, action)
        
        await limiter.unset_busy("AsyncRobot_apply_action")

        return action

    async def _reading_loop(self, driver_name: str) -> None:
        """
        Polling from the driver ASAP.
        """

        limiter = self.read_limiters[driver_name]

        while True:
            async with self.condition:
                state = self.state

            if state == RobotDeviceState.STOPPED:
                break

            start_time = TimeUtil.now()
            try:
                limiter = self.read_limiters[driver_name]
                await limiter.wait_for_tick("AsyncRobot_get_state")
                
                val = await AsyncUtil.run_blocking_as_async(self.robots[driver_name].get_state, timeout=self._timeout)

                # Write the input value to shared container
                # None generally means Communication Failure, ignore it
                if val is not None:
                    if self._input_queues[driver_name].full():
                        self._input_queues[driver_name].get_nowait()  # Discard the oldest value
                   
                    self._input_queues[driver_name].put_nowait(val)
                    # Broadcast the new value to all listeners
                    for listener in self._event_listeners[driver_name]:
                        AsyncUtil.detach_coroutine(listener(val))
                
                await limiter.unset_busy("AsyncRobot_get_state")

            except asyncio.CancelledError:
                await limiter.unset_busy("AsyncRobot_get_state")
                break  # Task was cancelled by some caller up in the call stack, exit
            except ReadError as e:
                await limiter.unset_busy("AsyncRobot_get_state")
                logger.error(f"Error reading from {driver_name}: {e}")
                raise e

            elapsed_time = TimeUtil.get_elapsed_time_ms(start_time)
            if DEBUG_ASYNC_INPUT:
                logger.info(
                    f"{driver_name}: Polling took {elapsed_time}ms({int(float(1000) / elapsed_time)} Hz)",
                )

    async def stop(self) -> None:
        """
        Gracefully stop the reading loop.
        You should invoke this method for stopping the input device for a specific driver,
        do not override this method!
        For resource freeing, override stop() instead for a specific driver
        """
        await self.set_state(RobotDeviceState.STOPPED)

        if (
            self._polling_read_tasks is not None
            and not await TimeUtil.wait_until_condition(
                condition=lambda: all(task.done() for task in self._polling_read_tasks.values()),
                timeout_ms=1000,
            )
        ):

            try:
                for task in self._polling_read_tasks.values():
                    task.cancel()
            except asyncio.CancelledError:
                pass
            except Exception as e:
                raise e

            self._polling_read_tasks = {}

        for driver_name in self.robots.keys():
            await AsyncUtil.run_blocking_as_async(self.robots[driver_name].stop)