import gymnasium as gym
import numpy as np
from gymnasium import spaces
import mujoco
from mujoco.viewer import launch_passive
from scipy.spatial.transform import Rotation as R
import yaml
import os
import importlib.resources as pkg_resources
from mini_ros.devices.motors.encoder import EncoderReader

class ExoG1Env(gym.Env):
    def __init__(self):
        super().__init__()
        
        assets_path = pkg_resources.files("we_sim") / "assets"
        xml_path = assets_path / "skeleton" / "G1.xml"
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data  = mujoco.MjData(self.model)
        
        self.action_space = spaces.Box(-1., 1., shape=(14,), dtype=np.float32)
        obs_dim = 14 + 14 + 12  # joint_pos + joint_vel + imu_data
        self.observation_space = spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)
        
        self.viewer = launch_passive(self.model, self.data,
                                     show_left_ui=False,
                                     show_right_ui=False)

    def reset(self, seed=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        # Zero out all joints
        self.data.qpos[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        self.data.qpos[0:14] = action
        # 3) Recompute body positions (no dynamics)
        mujoco.mj_forward(self.model, self.data)
        
        # 4) Render
        if self.viewer is not None:
            self.viewer.sync()
        
        # 5) Build obs & dummy reward/done
        obs = self._get_obs()
        return obs, 0.0, False, False, {}

    def _get_obs(self):
        # Joint positions and velocities
        joint_pos = self.data.qpos[0:14]
        joint_vel = self.data.qvel[0:14]
        
        # IMU data from sensor
        imu_linvel = self.data.sensordata[1:4]    # local linear velocity
        imu_accel  = self.data.sensordata[4:7]    # accelerometer
        imu_gyro   = self.data.sensordata[7:10]   # gyroscope
        imu_quat   = self.data.sensordata[10:14]  # orientation quaternion
        
        # Combine all observations
        return np.concatenate([
            joint_pos, joint_vel,
            imu_linvel, imu_accel,
            imu_gyro, imu_quat
        ]).astype(np.float32)

    def render(self):
        # passive viewer already synced in step()
        pass

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        
        # Close all encoder connections
        for enc in self.broadcast_encoders.values():
            try:
                enc.close()
            except:
                pass


if __name__ == "__main__":
    import time
    env = ExoG1Env()
    obs, _ = env.reset()
    
    print("G1 Skeleton Environment started")
    print(f"Observation shape: {obs.shape}")
    print(f"Action space: {env.action_space}")
    print("Press Ctrl+C to exit...")
    try:
        while True:
            obs, reward, done, truncated, info = env.step(None)
            

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        env.close() 