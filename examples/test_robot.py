import time
from loguru import logger
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.common.state import RobotAction
from mini_ros.wrapper.async_robot import AsyncRobot

def get_robot(robot_type: str, *args, **kwargs):
    if robot_type == "marvin":
        robot = MarvinRobot(MarvinRobotConfig(*args, **kwargs))
    elif robot_type == "pika":
        robot = PikaGripper(PikaGripperConfig(*args, **kwargs))
    else:
        raise ValueError(f"Invalid robot type: {robot_type}")
    return robot

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

    robot = get_robot("pika", port="/dev/ttyUSB0")
    robot.initialize()
    robot.start()
    logger.info("Robot Started.")
    for i in range(1000):
        action = RobotAction(
            timestamp=0,
            joint_cmds=[0.001 * i]
        )
        robot.apply_action(action)
        print(robot.get_state())
        time.sleep(0.01)
        
    robot.stop()

    #### 2. Test pika ####
    zero_action = RobotAction(timestamp=0, joint_cmds=0)