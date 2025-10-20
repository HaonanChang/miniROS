import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import importlib.resources as pkg_resources


class G1UpperBodyEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # Load the model
        assets_path = pkg_resources.files("we_sim") / "assets"
        xml_path = assets_path / "g1_description" / "g1_upper_body.xml"
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        
        self.action_range = np.copy(self.model.jnt_range)
        # Define action space (14 DOF for upper body + 6 DOF for torso)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(20,),  # 14 arm joints + 6 torso joints
            dtype=np.float32
        )
        
        # Define observation space
        # Joint positions (20), joint velocities (20), IMU data (3+3+3+3)
        obs_dim = 20 + 20 + 12  # positions + velocities + IMU data
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )
        
        # Initialize viewer
        self.viewer = None
        self.vis_torso_goal = True
        self.torso_goal = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0])

    def reset(self, seed=None):
        super().reset(seed=seed)
        
        # Reset the simulation
        mujoco.mj_resetData(self.model, self.data)
        
        # Set initial joint positions to neutral
        self.data.qpos[7:21] = 0.0  # Upper body joints
        
        # Forward simulation to update positions
        mujoco.mj_forward(self.model, self.data)
        
        return self._get_obs(), {}
    
    def step(self, action):
        # Clip action to [-1, 1]
        action = np.clip(action, -1, 1)
        
        # Scale actions to joint ranges
        scaled_action = self._scale_action(action)
        
        # Set control signals
        self.data.ctrl[:] = scaled_action
        
        # Update marker position
        self.torso_goal = scaled_action[0:6]  # torso_x, torso_y, torso_z, torso_roll, torso_pitch, torso_yaw
        z_offset = 0.5
        marker_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso_goal_marker")
        self.model.body_pos[marker_id] = np.array([self.torso_goal[0], self.torso_goal[1], self.torso_goal[2] + z_offset])
        # body_quat = R.from_euler('XYZ', self.torso_goal[3:6], degrees=False).as_quat(scalar_first=True)
        body_quat = R.from_euler('XYZ', self.torso_goal[3:6], degrees=False).as_quat()
        # Convert to scalar first
        body_quat = np.concatenate([body_quat[1:], [body_quat[0]]])
        self.model.body_quat[marker_id] = body_quat

        # Step the simulation
        mujoco.mj_step(self.model, self.data)
        
        # Get observation
        obs = self._get_obs()
        
        # Calculate reward (placeholder)
        reward = 0.0
        
        # Check if episode is done (placeholder)
        done = False
        
        return obs, reward, done, False, {}
    
    def _get_obs(self):
        # Get torso position and orientation
        torso_pos = self.data.qpos[0:6]
        torso_vel = self.data.qvel[0:6]
        
        # Get joint positions and velocities
        joint_pos = self.data.qpos[6:20]  # Upper body joints
        joint_vel = self.data.qvel[6:20]  # Upper body joint velocities
        
        # Get IMU data
        imu_linvel = self.data.sensordata[1:4]  # Linear velocity
        imu_accel = self.data.sensordata[4:7]   # Acceleration
        imu_gyro = self.data.sensordata[7:10]   # Angular velocity
        imu_quat = self.data.sensordata[10:14]  # Orientation quaternion
        
        # Combine all observations
        obs = np.concatenate([
            torso_pos,
            torso_vel,
            joint_pos,
            joint_vel,
            imu_linvel,
            imu_accel,
            imu_gyro,
            imu_quat
        ])
        
        return obs.astype(np.float32)
    
    def get_action_range(self):
        ranges = np.array([
            # Torso joints
            [-1.0, 1.0],      # torso_x
            [-1.0, 1.0],      # torso_y
            [-1.0, 1.0],      # torso_z
            [-100, 100],    # torso_yaw
            [-100, 100],    # torso_pitch
            [-100, 100],    # torso_roll
            # Left arm joints
            [-3.0892, 2.6704],  # left_shoulder_pitch
            [-1.5882, 2.2515],  # left_shoulder_roll
            [-2.618, 2.618],    # left_shoulder_yaw
            [-1.0472, 2.0944],  # left_elbow
            [-1.97222, 1.97222], # left_wrist_roll
            [-1.61443, 1.61443], # left_wrist_pitch
            [-1.61443, 1.61443], # left_wrist_yaw
            # Right arm joints
            [-3.0892, 2.6704],  # right_shoulder_pitch
            [-2.2515, 1.5882],  # right_shoulder_roll (different range)
            [-2.618, 2.618],    # right_shoulder_yaw
            [-1.0472, 2.0944],  # right_elbow
            [-1.97222, 1.97222], # right_wrist_roll
            [-1.61443, 1.61443], # right_wrist_pitch
            [-1.61443, 1.61443], # right_wrist_yaw
        ])
        return ranges
    
    def _scale_action(self, action):
        # Scale actions from [-1, 1] to actual joint ranges
        scaled_action = np.zeros_like(action)
        
        # Joint ranges from the XML file
        ranges = self.get_action_range()
        
        # Scale each joint's action
        for i in range(20):  # All 20 joints
            scaled_action[i] = (action[i] + 1) / 2 * (ranges[i, 1] - ranges[i, 0]) + ranges[i, 0]
        
        return scaled_action
    
    def render(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(
                model=self.model,
                data=self.data,
            )
        else:
            self.viewer.sync()
            
    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None 


if __name__ == "__main__":
    import time
    env = G1UpperBodyEnv()
    env.reset()
    env.render()
    while True:
        action = np.zeros(20)
        # action[0] = 1.0
        env.step(action)
        env.render()
        time.sleep(0.01)