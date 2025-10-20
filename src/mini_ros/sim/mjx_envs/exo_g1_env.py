import os
import time
import signal
import sys
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer


class ExoG1Env(gym.Env):
    """
    Exoskeleton for G1 Sim/RL Env
    """
    def __init__(self):
        super().__init__()
        
        root_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = f"{root_dir}/../../../../assets/exo_g1/G1.xml"
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data  = mujoco.MjData(self.model)
        
        self.action_space = spaces.Box(-1., 1., shape=(14,), dtype=np.float32)
        obs_dim = 14 + 14 + 12  # joint_pos + joint_vel + imu_data
        self.observation_space = spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)
        self.num_dof = 14

        # Initialize viewer and exit handling
        self.viewer = None
        self.should_exit = False

        # Set up signal handlers for graceful exit
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

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
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(
                model=self.model,
                data=self.data,
            )
            # Access and set the viewer camera
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.viewer.cam.lookat[:] = [0.0, 0, 0]
            self.viewer.cam.distance = 2.0
            self.viewer.cam.azimuth = -90
            self.viewer.cam.elevation = -20
            self.scene = self.viewer.user_scn
        else:
            self.viewer.sync()
            
        # Check if viewer was closed
        if self.viewer is not None and hasattr(self.viewer, 'is_running'):
            if not self.viewer.is_running():
                self.should_exit = True

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\nReceived signal {signum}. Shutting down gracefully...")
        self.should_exit = True
        self.close()
        sys.exit(0)


if __name__ == "__main__":
    env = ExoG1Env()
    env.reset()
    step_count = 0
    start_time = time.time()    
    while not env.should_exit:
        # Randomize action
        action = np.random.uniform(-1.0, 1.0, env.num_dof)
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