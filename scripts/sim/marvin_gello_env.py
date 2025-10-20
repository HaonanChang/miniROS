import os
import time
import numpy as np
import yaml
from tqdm import tqdm
from mini_ros.utils.robo_util import ForwardKinematics
from mini_ros.sim.mjx_envs.marvin_env import MarvinEnv
from mini_ros.devices.motors.feetech import FeetechReader
from mini_ros.common.device import motor_config_from_json


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.abspath(__file__))
    joint_map_file = f"{root_dir}/../../assets/gellos/joint_map_feetech.yaml"
    with open(joint_map_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)
    motor_config = motor_config_from_json(joint_config)
    gello = FeetechReader()
    gello.initialize(motor_config)

    env = MarvinEnv(is_new_version=True)
    env.reset()

    left_fk = ForwardKinematics(
        urdf_path=env.xml_path.replace(".xml", ".urdf"),
        base_link="World",
        ee_link="Link7_L"
    )
    right_fk = ForwardKinematics(
        urdf_path=env.xml_path.replace(".xml", ".urdf"),
        base_link="World",
        ee_link="Link7_R"
    )
    timeout = 40000
    for i in tqdm(range(timeout)):
        action = gello.get_state()
        # Check boundary
        left_ee_pos = left_fk.forward(action[:7][None, :])
        right_ee_pos = right_fk.forward(action[7:][None, :])
        if left_ee_pos[0, 0, 3] < -0.65 or left_ee_pos[0, 0, 3] > 0.65:
            raise ValueError("Left x pos is out of boundary")
        if right_ee_pos[0, 0, 3] < -0.65 or right_ee_pos[0, 0, 3] > 0.65:
            raise ValueError("Right x pos is out of boundary")
        env.step(action)
        env.render()
        time.sleep(0.01)
    env.close()