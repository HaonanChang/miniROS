"""
Environment for a dual-arm robot to pick up blocks and place them at target locations.
"""

import os
from copy import deepcopy
from datetime import datetime
import numpy as np
import mujoco
import mediapy as media
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import gymnasium as gym
import importlib.resources as pkg_resources
from gymnasium import spaces
from threading import Lock


class DualPiperBlockPickupEnv(gym.Env):
    """
    Environment for a dual-arm robot to pick up blocks and place them at target locations.
    """

    metadata = {"render_modes": ["human", "rgb_array", "mujoco_gui"], "render_fps": 60, "control_fps": 60}

    def __init__(
        self,
        render_mode: Optional[str] = None,
        max_episode_steps: int = 500,
        camera_view: str = "top-front",
        randomize_block_positions: bool = True,
        reward_type: str = "sparse",  # "sparse" or "dense"
    ):
        super().__init__(render_mode=render_mode, res_str="HD", max_episode_steps=max_episode_steps, num_dof=14)
        self.control_sub_steps: int = 1000 // self.metadata["control_fps"]
        self.control_per_render: int = self.metadata["control_fps"] // self.metadata["render_fps"]
        self.camera_view = camera_view
        self.randomize_block_positions = randomize_block_positions
        self.reward_type = reward_type
        self.images = {
            "overhead_cam": np.zeros((self.h, self.w, 3), dtype=np.uint8),
            "wrist_cam_left": np.zeros((self.h, self.w, 3), dtype=np.uint8),
            "wrist_cam_right": np.zeros((self.h, self.w, 3), dtype=np.uint8),
        }

        self._camera_io_lock = Lock()

        # Instead of using a relative path, use importlib.resources to locate the assets
        # Try to find the assets directory
        try:
            # First, check if we're running from the installed package
            assets_path = pkg_resources.files("we_sim") / "assets"
            model_xml = assets_path / "piper_xml" / "dual_piper_block_pickup_scene.xml"
            if not model_xml.exists():
                raise FileNotFoundError(f"Asset not found at {model_xml}")
        except (ImportError, FileNotFoundError):
            # Fallback: try relative to the current file
            current_dir = Path(__file__).parent.parent.parent  # Go up to the project root
            model_xml = current_dir / "assets" / "piper_xml" / "dual_piper_block_pickup_scene.xml"
            if not model_xml.exists():
                # One more fallback: try the original location
                model_xml = Path("./assets/piper_xml/dual_piper_block_pickup_scene.xml")
                if not model_xml.exists():
                    raise FileNotFoundError(f"Could not find assets at any location. Tried: {model_xml}")

        print(f"Loading model from: {model_xml}")
        self.model = mujoco.MjModel.from_xml_path(str(model_xml))
        self.data = mujoco.MjData(self.model)

        # Set up rendering
        self.model.vis.global_.offheight = self.h
        self.model.vis.global_.offwidth = self.w
        self.renderer = mujoco.Renderer(self.model, height=self.h, width=self.w)

        # Create a dictionary of cameras
        self.cameras = {}

        # Initialize cameras using the ones defined in the XML file
        for cam_name in ["overhead_cam", "wrist_cam_left", "wrist_cam_right"]:
            # for cam_name in ["overhead_cam"]:
            # Create a new camera object
            self.cameras[cam_name] = mujoco.MjvCamera()

            # Get the camera ID from the model
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)

            if cam_id >= 0:  # If camera exists in the model
                # Set the camera type to fixed (as defined in the XML)
                self.cameras[cam_name].type = mujoco.mjtCamera.mjCAMERA_FIXED
                # Set the camera ID to use the one from the XML
                self.cameras[cam_name].fixedcamid = cam_id
                print(f"Using camera '{cam_name}' from XML file (ID: {cam_id})")
            else:
                # Fallback to default free camera if not found in XML
                mujoco.mjv_defaultFreeCamera(self.model, self.cameras[cam_name])
                print(f"Warning: Camera '{cam_name}' not found in XML, using default free camera")

        # Rendering options
        self.vis = mujoco.MjvOption()
        self.vis.geomgroup[2] = True  # Visual geometries
        self.vis.geomgroup[3] = False  # Collision geometries

        # Define action and observation spaces
        # Action space: 16 joint positions (8 for each arm)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(16,), dtype=np.float32)

        # Observation space: joint positions, velocities, block positions, target positions, and camera image
        self.observation_space = spaces.Dict(
            {
                "joint_positions": spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float32),
                "joint_velocities": spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float32),
                "block_positions": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),  # 1 block x 3 coordinates
                "target_positions": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),  # 1 target x 3 coordinates
                "gripper_positions": spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32),  # 2 grippers x 3 coordinates
                "image": spaces.Box(low=0, high=255, shape=(self.h, self.w, 3), dtype=np.uint8),  # RGB image from topdown camera
            }
        )

        # Block and target information
        self.block_names = ["red_block"]  # Only one block
        self.target_names = ["target_red"]  # Only one target

        # For human rendering
        self.window = None
        self.clock = None

        # For MuJoCo GUI rendering
        self.mujoco_viewer = None

        # Initialize the simulation
        mujoco.mj_forward(self.model, self.data)

    def _get_obs(self) -> Dict[str, np.ndarray]:
        """Get the current observation."""
        # Get joint positions and velocities
        joint_positions = np.zeros(16)
        joint_velocities = np.zeros(16)

        # Left arm joints (0-7)
        for i in range(8):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"left/joint{i+1}")
            joint_positions[i] = self.data.qpos[joint_id]
            joint_velocities[i] = self.data.qvel[joint_id]

        # Right arm joints (8-15)
        for i in range(8):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"right/joint{i+1}")
            joint_positions[i + 8] = self.data.qpos[joint_id]
            joint_velocities[i + 8] = self.data.qvel[joint_id]

        # Get block positions
        block_positions = np.zeros(3)  # Changed from 9 to 3
        for i, block_name in enumerate(self.block_names):
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, block_name)
            block_pos = self.data.xpos[body_id]
            block_positions[i * 3 : (i + 1) * 3] = block_pos

        # Get target positions
        target_positions = np.zeros(3)  # Changed from 9 to 3
        for i, target_name in enumerate(self.target_names):
            site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, target_name)
            target_pos = self.data.site_xpos[site_id]
            target_positions[i * 3 : (i + 1) * 3] = target_pos

        # Get gripper positions
        gripper_positions = np.zeros(6)
        left_gripper_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left/link7")
        right_gripper_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right/link7")
        gripper_positions[0:3] = self.data.xpos[left_gripper_body]
        gripper_positions[3:6] = self.data.xpos[right_gripper_body]

        # Capture images from all cameras
        # Use cached images if in async render mode
        color_images = deepcopy(self.images)

        # Timestamp
        timestamp = datetime.now().timestamp()

        return {
            "joint_positions": joint_positions.astype(np.float32),
            "joint_velocities": joint_velocities.astype(np.float32),
            "block_positions": block_positions.astype(np.float32),
            "target_positions": target_positions.astype(np.float32),
            "gripper_positions": gripper_positions.astype(np.float32),
            "color_images": color_images,  # Dictionary of images from all cameras
            "timestamp": timestamp,
        }

    def _get_joint_state(self) -> np.ndarray:
        """Get the joint state (pos+vel) of the environment."""
        joint_states = np.zeros(32)
        # Left arm joints (0-7)
        for i in range(7):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"left/joint{i+1}")
            joint_states[i] = self.data.qpos[joint_id]
            joint_states[14 + i] = self.data.qvel[joint_id]

        # Right arm joints (7-14)
        for i in range(7):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"right/joint{i+1}")
            joint_states[i + 7] = self.data.qpos[joint_id]
            joint_states[14 + i + 7] = self.data.qvel[joint_id]
        return joint_states

    def _get_info(self) -> Dict[str, Any]:
        """Get additional information about the environment state."""
        info = {}

        # Calculate distances between blocks and their targets
        for i, (block_name, target_name) in enumerate(zip(self.block_names, self.target_names)):
            block_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, block_name)
            target_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, target_name)

            block_pos = self.data.xpos[block_body_id]
            target_pos = self.data.site_xpos[target_site_id]

            distance = np.linalg.norm(block_pos - target_pos)
            info[f"{block_name}_distance"] = distance
            info[f"{block_name}_at_target"] = distance < 0.05  # 5cm threshold

        # Calculate success rate
        blocks_at_target = sum(info[f"{block_name}_at_target"] for block_name in self.block_names)
        info["success_rate"] = blocks_at_target / len(self.block_names)
        info["success"] = info["success_rate"] == 1.0

        return info

    def _get_reward(self, info: Dict[str, Any]) -> float:
        """Calculate the reward based on the current state."""
        if self.reward_type == "sparse":
            # Sparse reward: 1.0 if all blocks are at their targets, 0.0 otherwise
            return float(info["success"])
        else:  # Dense reward
            # Dense reward: negative sum of distances between blocks and targets
            # Plus a bonus for each block at its target
            reward = 0.0
            for block_name in self.block_names:
                distance = info[f"{block_name}_distance"]
                reward -= distance  # Negative distance (closer is better)
                if info[f"{block_name}_at_target"]:
                    reward += 1.0  # Bonus for each block at target
            return reward

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """Reset the environment to an initial state."""
        super().reset(seed=seed)

        # Reset the simulation
        mujoco.mj_resetData(self.model, self.data)

        # Randomize block positions if enabled
        if self.randomize_block_positions:
            for block_name in self.block_names:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, block_name)
                # Random position on the table
                x = self.np_random.uniform(-0.1, 0.1)
                y = self.np_random.uniform(-0.1, 0.1)
                z = 0.025  # Height of block center above table

                # Set the position in qpos (7 values: 3 for position, 4 for quaternion)
                qpos_start = self.model.jnt_qposadr[self.model.body_jntadr[body_id]]
                self.data.qpos[qpos_start : qpos_start + 3] = [x, y, z]
                # Keep default orientation (identity quaternion)
                self.data.qpos[qpos_start + 3 : qpos_start + 7] = [1, 0, 0, 0]

        # Reset step counter
        self.steps = 0

        # Forward the simulation to update positions
        mujoco.mj_forward(self.model, self.data)

        # Get observation and info
        observation = self._get_obs()
        info = self._get_info()

        # Note: we don't render here, as we want the render happends only within the step function
        return observation, info

    def step(self, action: np.ndarray = None) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """Take a step in the environment."""

        # If no action is provided, use the current action
        if action is None:
            action = deepcopy(self.cur_action)

        # Pad extra gripper value for action
        # As in real control, both gripper joint is controlled by a single value
        if action.shape == (14,):
            action = np.concatenate([action[0:7], action[6:7], action[7:14], action[13:14]])

        # Apply action
        self.data.ctrl[:] = action

        # Step the simulation
        for _ in range(self.control_sub_steps):  # Multiple steps for stability
            mujoco.mj_step(self.model, self.data)
        # Get observation, reward, and info
        observation = self._get_obs()
        info = self._get_info()
        reward = self._get_reward(info)

        # Check if episode is done
        terminated = info["success"]  # Changed from always False to terminated on success
        truncated = self.steps >= self.max_episode_steps

        # Render if in human mode or mujoco_gui mode
        if self.steps % self.control_per_render == 0:
            self._render_frame()
            if self.render_mode == "mujoco_gui":
                self._render_mujoco_gui()

        # Update step counter
        self.steps += 1
        return observation, reward, terminated, truncated, info

    def render(self) -> Optional[np.ndarray]:
        """Render the environment."""
        if self.render_mode == "rgb_array":
            return self._render_frame()
        elif self.render_mode == "mujoco_gui":
            self._render_frame()
            self._render_mujoco_gui()
            return None

        return None

    def get_frames(self) -> Dict[str, np.ndarray]:

        ret = None

        with self._camera_io_lock:
            ret = deepcopy(self.images)

        return ret

    def _render_frame(self) -> Optional[np.ndarray]:
        """Render the current frame."""
        # Update scene and render for all cameras

        with self._camera_io_lock:
            for cam_name, camera in self.cameras.items():
                self.renderer.update_scene(self.data, camera, scene_option=self.vis)
                frame = self.renderer.render().copy()
                self.images[cam_name] = frame

        # For human rendering, show the overhead camera view
        if self.render_mode == "human":
            media.show_image(self.images["overhead_cam"])
            time.sleep(1 / 60)  # Approximate 60 FPS

        return self.images["overhead_cam"]  # Return the overhead view by default

    def _render_mujoco_gui(self) -> None:
        """Render using MuJoCo's built-in GUI viewer."""
        try:
            import mujoco.viewer
        except ImportError:
            raise ImportError("To use MuJoCo GUI rendering, you need to have mujoco.viewer available.")

        if self.mujoco_viewer is None:
            try:
                self.mujoco_viewer = mujoco.viewer.launch_passive(self.model, self.data)
            except RuntimeError as e:
                if "mjpython" in str(e) and "macOS" in str(e):
                    print("Error: On macOS, you must run this script using 'mjpython' instead of regular python.")
                    print("Try: mjpython your_script.py")
                    # Fall back to rgb_array rendering
                    self.render_mode = "rgb_array"
                    return self._render_frame()
                else:
                    raise e

        if self.mujoco_viewer is not None and self.mujoco_viewer.is_running():
            self.mujoco_viewer.sync()
        else:
            self.mujoco_viewer = None

    def close(self) -> None:
        """Clean up resources."""
        if self.window is not None:
            # Close window if using pygame or other window-based rendering
            pass

        if self.mujoco_viewer is not None:
            self.mujoco_viewer.close()
            self.mujoco_viewer = None


# Example usage
if __name__ == "__main__":
    import mediapy as media
    import os

    # Create output directory if it doesn't exist
    output_dir = "camera_recordings"
    os.makedirs(output_dir, exist_ok=True)

    # Initialize environment
    env = DualPiperBlockPickupEnv(render_mode="mujoco_gui", camera_view="top-front")
    observation, info = env.reset()

    # Parameters for the sinusoidal movement
    amplitude = 0.5  # Maximum joint angle deviation
    period = 10  # Steps for one complete cycle
    num_cycles = 2  # Number of cycles per joint
    num_joints = 16  # Total number of joints
    fps = 30  # Frame rate for the output videos

    # Initialize video buffers for each camera
    video_buffers = {cam_name: [] for cam_name in env.cameras.keys()}

    # For each joint
    for joint in range(num_joints):
        print(f"Moving joint {joint}")

        # For each cycle of the current joint
        for step in range(period * num_cycles):
            # Create a zero action vector
            action = np.zeros(16)

            # Apply sinusoidal movement to only the current joint
            action[joint] = amplitude * np.sin(2 * np.pi * step / period)

            # Take the action
            observation, reward, terminated, truncated, info = env.step(action)

            # Store frames from all cameras
            for cam_name, frame in observation["color_images"].items():
                video_buffers[cam_name].append(frame)

            if terminated or truncated:
                observation, info = env.reset()

        # Reset the environment after testing each joint
        observation, info = env.reset()

    # Save videos for each camera
    print("Saving camera recordings...")
    for cam_name, frames in video_buffers.items():
        if frames:  # Check if we have any frames
            frames = np.stack(frames)  # Stack frames into a single array
            output_path = os.path.join(output_dir, f"{cam_name}.mp4")

            # Save as MP4 using mediapy
            media.write_video(output_path, frames, fps=fps)
            print(f"Saved {cam_name} recording to {output_path}")

    env.close()
