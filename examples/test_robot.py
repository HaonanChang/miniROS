import time
from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
from mini_ros.wrapper.async_robot import AsyncRobot

def get_robot(robot_type: str):
    if robot_type == "marvin":
        robot = MarvinRobot(MarvinRobotConfig())
    elif robot_type == "pika":
        robot = PikaGripper(PikaGripperConfig())
    else:
        raise ValueError(f"Invalid robot type: {robot_type}")
    return robot

if __name__ == "__main__":
    robot = get_robot("marvin")
    robot.initialize()
    robot.start()
    for i in range(100):
        robot.get_state()
        time.sleep(0.01)
        print(robot.get_state())
    robot.stop()