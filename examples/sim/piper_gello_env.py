"""
Use Gello's action to control the exoskeleton.
"""
import os
import time
import signal
import sys
import numpy as np
import yaml
from mini_ros.sim.mjx_envs.piper_env import PiperEnv
from mini_ros.devices.motors.dynamixel import DynamixelReader
from mini_ros.common.device import motor_config_from_json


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.abspath(__file__))
    joint_map_file = f"{root_dir}/../../assets/gellos/joint_map_dynamixel.yaml"
    with open(joint_map_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)
    motor_config = motor_config_from_json(joint_config)
    
    gello = DynamixelReader()
    gello.initialize(motor_config)

    env = PiperEnv()
    env.reset()
    step_count = 0
    start_time = time.time()    
    while not env.should_exit:
        # Randomize action
        action = gello.get_state()
        env.step(action)
        env.render()
        step_count += 1

        # Print periodic status updates
        if step_count % 1000 == 0:
            elapsed = time.time() - start_time
            hz = step_count / elapsed
            print(f"[Info]: Steps: {step_count}, Rate: {hz:.1f} Hz")
        
        time.sleep(0.01)
    env.close()