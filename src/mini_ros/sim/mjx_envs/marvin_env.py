import gymnasium as gym
import numpy as np
from gymnasium import spaces
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import signal
import sys
import os
import time
from typing import List, Dict, Tuple, Optional


class MarvinEnv(gym.Env):
    """
    Marvin Sim/RL Env
    """
    def __init__(self, is_new_version: bool = False):
        super().__init__()
        self.is_new_version = is_new_version

        # Load the marvin model
        root_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = f"{root_dir}/../../../../assets/tj_arm/urdf/Marvin-BiArm-Baseless.xml"
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        self.action_range = np.copy(self.model.jnt_range)
        # Define action space for marvin robot (bilateral arms, no hand)
        # 7 left arm DOF + 7 right arm DOF = 14 total DOF
        self.num_dof = 14
        self.left_arm_dof = 7    # L_arm_j1 through L_arm_j7
        self.right_arm_dof = 7   # R_arm_j1 through R_arm_j7
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.num_dof,),
            dtype=np.float32
        )

        self.ref_qpos = np.array([
            # Left arm
            90, -90, -90, -90, 90, 0, 0, 
            # Right arm
            -90, -90, 90, -90, -90, 0, 0,
        ]) * np.pi / 180.0

        # Define observation space
        # Joint positions (14) + joint velocities (14) + IMU data (12) = 40 dimensions
        obs_dim = self.num_dof + self.num_dof + 12  # positions + velocities + IMU data
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )

        # Initialize viewer and exit handling
        self.viewer = None
        self.should_exit = False

        # Set up signal handlers for graceful exit
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def reset(self, seed=None):
        super().reset(seed=seed)
        
        # Reset the simulation
        mujoco.mj_resetData(self.model, self.data)
        
        # Set initial joint positions to neutral
        # All joints start at 0 (neutral position)
        self.data.qpos[:] = 0.0
        
        # Forward simulation to update positions
        mujoco.mj_forward(self.model, self.data)
        
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        if action.shape[-1] == 16:
            # Extract left and right arm actions
            left_arm_action = action[:7]
            right_arm_action = action[8:15]
            # Project action
            left_arm_action = self._joint_project_clip(left_arm_action, self.is_new_version)
            right_arm_action = self._joint_project_clip(right_arm_action, self.is_new_version)
            self.data.qpos[:7] = left_arm_action
            self.data.qpos[7:] = right_arm_action
        elif action.shape[-1] == 14:
            # Set joint positions
            self.data.qpos[:] = action
        else:
            raise ValueError(f"Invalid action shape: {action.shape}")
        
        # Forward simulation to update positions
        mujoco.mj_forward(self.model, self.data)
        
        # Get observation
        obs = self._get_obs()
        
        # Calculate reward (placeholder)
        reward = 0.0
        
        # Check if episode is done (placeholder)
        done = False
        
        return obs, reward, done, False, {}

    def _get_obs(self):
        # Get all joint positions (14 DOF: 7 left arm + 7 right arm, no hand)
        joint_pos = self.data.qpos[:self.num_dof] if self.model.nq >= self.num_dof else np.zeros(self.num_dof)
        
        # Get all joint velocities
        joint_vel = self.data.qvel[:self.num_dof] if self.model.nv >= self.num_dof else np.zeros(self.num_dof)
        
        # Get IMU data (torso sensor data if available)
        imu_data = np.zeros(12)  # Default to zeros if no IMU
        if len(self.data.sensordata) >= 12:
            imu_accel = self.data.sensordata[0:3]     # Acceleration
            imu_gyro = self.data.sensordata[3:6]      # Angular velocity  
            imu_linvel = self.data.sensordata[6:9]    # Linear velocity
            imu_quat = self.data.sensordata[9:13]     # Orientation quaternion
            imu_data = np.concatenate([imu_accel, imu_gyro, imu_linvel, imu_quat])
        
        # Combine all observations: joint_pos + joint_vel + imu_data
        obs = np.concatenate([
            joint_pos,
            joint_vel,
            imu_data
        ])
        
        return obs.astype(np.float32)
    
    def _scale_action(self, action):
        # Scale actions from [-1, 1] to actual joint ranges
        scaled_action = np.zeros_like(action)
        
        # Joint ranges
        ranges = self.action_range
        
        # Scale each joint's action
        for i in range(min(len(action), len(ranges))):
            scaled_action[i] = (action[i] + 1) / 2 * (ranges[i, 1] - ranges[i, 0]) + ranges[i, 0]
        
        return scaled_action

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\nReceived signal {signum}. Shutting down gracefully...")
        self.should_exit = True
        self.close()
        sys.exit(0)
    
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
    
    def _joint_project_clip(self, joints_rad, is_new_version: bool = False):
        """
        Project joints on the joint boundary. Dependend on the version, we may have different methods.
        """
        ## Perform j6-j7 joint clipping
        j6 = joints_rad[5]
        j7 = joints_rad[6]
        j6 = np.clip(j6, -np.pi / 3, np.pi / 3)
        j7 = np.clip(j7, -np.pi / 2, np.pi / 2)
        j6_sign = 1 if j6 >= 0 else -1
        j7_sign = 1 if j7 >= 0 else -1
        X0 = np.abs(j6)
        Y0 = np.abs(j7)

        # Linear range: Y = AX + B:
        B = 78 / 180 * np.pi if not is_new_version else 109 / 180 * np.pi
        A = -1

        if Y0 > (A * X0 + B):
            # need projection
            X = (A * Y0 + X0 - A * B) / (A**2 + 1)
            Y = A * X + B
            if X < 0:
                X = 0
                Y = B
            elif Y < 0:
                X = -B / A
                Y = 0
            elif X < 0 and Y < 0:
                raise ValueError("Theoretically impossible.")
            j6 = j6_sign * X
            j7 = j7_sign * Y
        else:
            j6 = j6_sign * X0
            j7 = j7_sign * Y0
        joints_rad[5] = j6
        joints_rad[6] = j7
        return joints_rad


if __name__ == "__main__":
    env = MarvinEnv()
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