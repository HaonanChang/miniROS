from mini_ros.network.network_queue import One2ManySender
import time
import numpy as np
from mini_ros.common.state import RobotState, RobotAction
from mini_ros.system.robot_frequency_test import create_timed_joint_cmds_traj
from mini_ros.utils.rate_limiter import RateLimiterSync
phase_time = 2.0  # seconds

START_POSE = {
    "marvin": np.array([90.0, -70.0, -90.0, -110.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -110.0, -90.0, 0.0, 0.0]),
    "pika": np.array([0.0]),
}

END_POSE = {
    "marvin": np.array([90.0, -70.0, -90.0, -90.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -90.0, -90.0, 0.0, 0.0]),
    "pika": np.array([0.5]),
}

send_freq = 60
joint_cmds_traj = {
        "marvin": create_timed_joint_cmds_traj(start_joint_point=START_POSE["marvin"], end_joint_point=END_POSE["marvin"], num_points=int(phase_time * send_freq), num_repeat=6),
        "pika-0": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * send_freq), num_repeat=6),
        "pika-1": create_timed_joint_cmds_traj(start_joint_point=START_POSE["pika"], end_joint_point=END_POSE["pika"], num_points=int(phase_time * send_freq), num_repeat=6),
    }

robot_action = RobotAction(
    timestamp=time.time(),
    joint_cmds=[1, 2, 3, 4, 5],
    end_effector_cmds=[1, 2, 3, 4, 5],
    base_pose_cmds=[1, 2, 3, 4, 5, 6],
    base_velocity_cmds=[1, 2, 3, 4, 5, 6],
)

queue = One2ManySender(name="sender", port=5555, data_type="dict")


send_rate_limiter = RateLimiterSync(send_freq)
step_idx = 0
codes = ["IMPEDANCE", "STOP"]
while True:
    if step_idx >= len(joint_cmds_traj["marvin"]):
        break
    print(f"Step index: {step_idx}, Code: {codes[(step_idx // 100) % len(codes)]}")
    code = codes[(step_idx // 200) % len(codes)]
    send_rate_limiter.wait_for_tick()
    marvin_action = RobotAction(joint_cmds=joint_cmds_traj["marvin"][step_idx], code=code)
    pika_0_action = RobotAction(joint_cmds=joint_cmds_traj["pika-0"][step_idx], code=code)
    pika_1_action = RobotAction(joint_cmds=joint_cmds_traj["pika-1"][step_idx], code=code)
    queue.put({
        "marvin": marvin_action.asdict(),
        "pika-0": pika_0_action.asdict(),
        "pika-1": pika_1_action.asdict(),
    }, code="normal")
    step_idx += 1
    