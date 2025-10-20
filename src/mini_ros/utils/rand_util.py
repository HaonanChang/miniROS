import numpy as np


def rand_quat():
    """
    Generate a random quaternion.
    """
    qwxyz = np.random.rand(4)
    if np.linalg.norm(qwxyz) == 0:
        qwxyz = np.array([1, 0, 0, 0])
    else:
        qwxyz /= np.linalg.norm(qwxyz)
    return qwxyz


def rand_pose():
    """
    Generate a random pose.
    """
    pos = np.random.rand(3)
    quat = rand_quat()
    return np.concatenate([pos, quat])