"""Differentiable Forward kinematics."""

import numpy as np
from urchin import URDF
from scipy.spatial.transform import Rotation as R


###################################### FK #######################################
def skew_symmetric_matrix(axis):
    """Generate skew-symmetric matrix for rotation axis"""
    K = np.zeros((3, 3))
    K[0, 1] = -axis[2]
    K[0, 2] = axis[1]
    K[1, 0] = axis[2]
    K[1, 2] = -axis[0]
    K[2, 0] = -axis[1]
    K[2, 1] = axis[0]
    return K


class ForwardKinematics:
    def __init__(self, urdf_path, base_link, ee_link):
        # Load URDF and build kinematic chain
        self.urdf = URDF.load(urdf_path)
        self.base_link = base_link
        self.ee_link = ee_link

        # Build parent joint map and find kinematic chain
        self.parent_map = self._build_parent_map()
        self.chain_joints = self._find_chain()

        # Extract joint parameters
        self.transforms = []
        self.joint_axes = []
        self.joint_types = []
        self.joint_names = []
        self.active_joints = []

        # Preprocess joints in the chain
        for joint in self.chain_joints:
            # Store fixed transform from joint origin
            origin = np.array(joint.origin, dtype=np.float32)
            self.transforms.append(origin)

            # Store joint properties
            self.joint_types.append(joint.joint_type)
            if joint.joint_type in ["revolute", "prismatic", "continuous"]:
                axis = np.array(joint.axis, dtype=np.float32)
                axis /= np.linalg.norm(axis)  # Normalize
                self.joint_axes.append(axis)
                self.active_joints.append(joint)
            self.joint_names.append(joint.name)
        self.num_joints = len(self.active_joints)
        # Release URDF
        del self.urdf

    def _build_parent_map(self):
        """Create mapping from child links to their parent joints"""
        return {j.child: j for j in self.urdf.joints}

    def _find_chain(self):
        """Find joint sequence from base to end effector"""
        chain = []
        current = self.urdf.link_map[self.ee_link]

        while current.name != self.base_link:
            if current.name not in self.parent_map:
                raise ValueError(f"No path from {self.base_link} to {self.ee_link}")
            parent_joint = self.parent_map[current.name]
            # import ipdb; ipdb.set_trace()
            chain.append(parent_joint)
            current = self.urdf.link_map[parent_joint.parent]

        # Reverse to get base-to-ee order
        return list(reversed(chain))

    def forward(self, joint_positions: np.ndarray):
        """
        Args:
            joint_positions: (B, J)
        Returns:
            T: (B, 4, 4)
        """
        batch_size = joint_positions.shape[0]

        # Initialize transform with identity matrix
        T = np.tile(np.eye(4), (batch_size, 1, 1))

        joint_idx = 0  # Index for active joints

        for i, joint in enumerate(self.chain_joints):
            # Get fixed transform from URDF
            fixed_tf = self.transforms[i]
            fixed_tf = np.tile(fixed_tf, (batch_size, 1, 1))

            # Apply joint transform if active
            if joint.joint_type in ["revolute", "continuous"]:
                theta = joint_positions[:, joint_idx]
                axis = self.joint_axes[joint_idx]

                # Compute rotation using Rodrigues' formula
                K = skew_symmetric_matrix(axis)
                I = np.eye(3)
                theta = theta.reshape(-1, 1, 1)
                R = I + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K

                joint_tf = np.tile(np.eye(4), (batch_size, 1, 1))
                joint_tf[:, :3, :3] = R
                joint_idx += 1

            elif joint.joint_type == "prismatic":
                d = joint_positions[:, joint_idx]
                axis = self.joint_axes[joint_idx]

                joint_tf = np.tile(np.eye(4), (batch_size, 1, 1))
                joint_tf[:, :3, 3] = d.reshape(-1, 1) * axis
                joint_idx += 1

            else:
                joint_tf = np.tile(np.eye(4), (batch_size, 1, 1))

            # Compose transformations
            T = T @ fixed_tf @ joint_tf

        return T


###################################### Pose #######################################

def compute_relative_pose(pose_p, pose_c):
    """
    Compute the relative pose between two poses: pose_c (child) in pose_p (parent) frame.
    Pose is in the form of [x, y, z, qw, qx, qy, qz]
    """
    pose_mat_p = np.eye(4)
    pose_mat_p[:3, :3] = R.from_quat(pose_p[3:], scalar_first=True).as_matrix()
    pose_mat_p[:3, 3] = pose_p[:3]
    pose_mat_c = np.eye(4)
    pose_mat_c[:3, :3] = R.from_quat(pose_c[3:], scalar_first=True).as_matrix()
    pose_mat_c[:3, 3] = pose_c[:3]
    
    pose_mat_c_p = np.linalg.inv(pose_mat_p) @ pose_mat_c
    return np.concatenate([pose_mat_c_p[:3, 3], R.from_matrix(pose_mat_c_p[:3, :3]).as_quat(scalar_first=True)])
