from mini_ros.devices.robots.marvin_robot import MarvinRobot, MarvinRobotConfig
from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig

robot = MarvinRobot(MarvinRobotConfig(control_mode="impedance"))
robot.initialize()
robot.start()
robot.stop()