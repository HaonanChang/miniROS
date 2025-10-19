import time
import asyncio
from loguru import logger
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.common.state import RobotAction
from mini_ros.wrapper.async_robot import AsyncRobot
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.time_util import TimeUtil


def get_robot(robot_type: str, *args, **kwargs):
    if robot_type == "marvin":
        robot = MarvinRobot(MarvinRobotConfig(*args, **kwargs))
    elif robot_type == "pika":
        robot = PikaGripper(PikaGripperConfig(*args, **kwargs))
    else:
        raise ValueError(f"Invalid robot type: {robot_type}")
    return robot

async def test_pikas():
    robot_1 = get_robot("pika", port="/dev/ttyUSB0")
    robot_2 = get_robot("pika", port="/dev/ttyUSB1")

    # Initialize 
    await asyncio.gather(
        *[
            AsyncUtil.run_blocking_as_async(robot_1.initialize),
            AsyncUtil.run_blocking_as_async(robot_2.initialize),
        ]
    )

    # Start
    await asyncio.gather(
        *[
            AsyncUtil.run_blocking_as_async(robot_1.start),
            AsyncUtil.run_blocking_as_async(robot_2.start),
        ]
    )

    # Read
    for i in range(200):
        result = await asyncio.gather(
            *[
                AsyncUtil.run_blocking_as_async(robot_1.get_state),
                AsyncUtil.run_blocking_as_async(robot_2.get_state),
            ]
        )
        print(f"{i}: {result[0]}, {result[1]}")
        await TimeUtil.sleep_by_ms(1)

    await asyncio.gather(
            *[
                AsyncUtil.run_blocking_as_async(robot_1.stop),
                AsyncUtil.run_blocking_as_async(robot_2.stop),
            ]
        )

if __name__ == "__main__":

    #### 1. Test marvin ####
    # zero_action = RobotAction(
    #     timestamp=0,
    #     joint_cmds=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # )
    # default_action = RobotAction(
    #     timestamp=0,
    #     joint_cmds=[90.0, -70.0, -90.0, -90.0, 0.0, 0.0, 0.0, -90.0, -70.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    # )

    # zero_action = RobotAction(timestamp=0, joint_cmds=[0.2])
    # robot = get_robot("marvin")
    # robot = get_robot("pika", port="/dev/ttyUSB0")
    # robot.initialize()
    # robot.start()
    # logger.info("Robot Started.")
    # for i in range(100):
    #     # action = RobotAction(
    #     #     timestamp=0,
    #     #     joint_cmds=[1 - 0.001 * i]
    #     # )
    #     # robot.apply_action(action)
    #     print(robot.get_state())
    #     time.sleep(0.01)
        
    # robot.stop()

    # #### 2. Test pika ####
    # zero_action = RobotAction(timestamp=0, joint_cmds=0)

    asyncio.run(test_pikas())