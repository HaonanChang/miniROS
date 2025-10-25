from mini_ros.network.network_queue import One2ManySender
import time
import numpy as np
from mini_ros.common.state import RobotState, RobotAction

robot_action = RobotAction(
    timestamp=time.time(),
    joint_cmds=[1, 2, 3, 4, 5],
    end_effector_cmds=[1, 2, 3, 4, 5],
    base_pose_cmds=[1, 2, 3, 4, 5, 6],
    base_velocity_cmds=[1, 2, 3, 4, 5, 6],
)

queue = One2ManySender(name="sender", port=5555, data_type="dict")

while True:
    queue.put({
        "marvin": robot_action.asdict(),
        "pika-0": robot_action.asdict(),
        "pika-1": robot_action.asdict(),
    }, code="active")
    time.sleep(0.01)