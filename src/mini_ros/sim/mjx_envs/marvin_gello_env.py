import os
import time
import numpy as np
import yaml
from tqdm import tqdm
from mini_ros.utils.robo_util import ForwardKinematics
from mini_ros.sim.mjx_envs.marvin_env import MarvinEnv
from mini_ros.devices.motors.feetech import FeetechReader, motor_config_from_json
from mini_ros.devices.motors.dynamixel import DynamixelReader


class MarvinGelloEnv(MarvinEnv):
    """
    Capture marvin's action with gello's action.
    """
    def __init__(self, is_new_version: bool = False, gello_type="feetech", joint_config=None):
        super().__init__(is_new_version=is_new_version)
        if gello_type == "feetech":
            self.gello = FeetechReader()
        elif gello_type == "dynamixel":
            self.gello = DynamixelReader()
        else:
            raise ValueError(f"Invalid gello type: {gello_type}")
        self.gello.initialize(joint_config)
        # Add forward kinematics
        root_dir = os.path.dirname(os.path.abspath(__file__))
        self.left_fk = ForwardKinematics(
            urdf_path=f"{root_dir}/../../assets/tj_arm/urdf/Marvin-BiArm-Baseless.urdf",
            base_link="World",
            ee_link="Link7_L"
        )
        self.right_fk = ForwardKinematics(
            urdf_path=f"{root_dir}/../../assets/tj_arm/urdf/Marvin-BiArm-Baseless.urdf",
            base_link="World",
            ee_link="Link7_R"
        )
        self.marker_added = False

    def step(self, action):
        action = self.gello.get_state()
        # Add FK boundary check
        super().step(action)
        # if not self._check_fk_boundary():
        #     raise ValueError("Action is out of boundary")

    def _check_fk_boundary(self):
        qpos = self.data.qpos
        left_ee_pos = self.left_fk.forward(qpos[:7][None, :])
        right_ee_pos = self.right_fk.forward(qpos[7:][None, :])
        time_stamp = time.time()
        # Control x range
        if left_ee_pos[0, 0, 3] < -0.65 or left_ee_pos[0, 0, 3] > 0.65:
            return False
        # Control x range
        if right_ee_pos[0, 0, 3] < -0.65 or right_ee_pos[0, 0, 3] > 0.65:
            return False
        print(f"Left x pos: {left_ee_pos[0, 0, 3]}, Right x pos: {right_ee_pos[0, 0, 3]}")
        print(f"Left y pos: {left_ee_pos[0, 1, 3]}, Right y pos: {right_ee_pos[0, 1, 3]}")
        print(f"Left z pos: {left_ee_pos[0, 2, 3]}, Right z pos: {right_ee_pos[0, 2, 3]}")
        print(f"Time taken: {time.time() - time_stamp}")
        return True

    def render(self):
        super().render()


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.abspath(__file__))
    joint_map_file = f"{root_dir}/../../assets/gellos/joint_map_feetch_calib.yaml"
    with open(joint_map_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)
    motor_config = motor_config_from_json(joint_config)
    
    env = MarvinGelloEnv(is_new_version=True, gello_type="feetech", joint_config=motor_config)
    env.reset()
    timeout = 40000
    for i in tqdm(range(timeout)):
        env.step(np.zeros(16))
        env.render()
        time.sleep(0.01)
    env.close()