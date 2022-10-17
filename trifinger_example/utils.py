import numpy as np
import quaternion

from scipy.spatial.transform import Rotation

import trifinger_simulation.tasks.move_cube as task


def to_quat(x):
    """Numpy array to quaternion."""
    return np.quaternion(x[3], x[0], x[1], x[2])


def to_world_space(x_local, pose):
    """Transform from local space of object with given pose to world space."""

    q_rot = to_quat(pose.orientation)
    transl = pose.position
    q_local = np.quaternion(0., x_local[0], x_local[1], x_local[2])
    q_global = q_rot*q_local*q_rot.conjugate()
    return transl + np.array([q_global.x, q_global.y, q_global.z])


def random_yaw_orientation():
    """Random orientation for cube on floor."""

    # first "roll the die" to see which face is pointing upward
    up_face = np.random.choice(range(len(task._base_orientations)))
    up_face_rot = task._base_orientations[up_face]
    # then draw a random yaw rotation
    yaw_angle = np.random.uniform(0, 2 * np.pi)
    yaw_rot = Rotation.from_euler("z", yaw_angle)
    # and combine them
    orientation = yaw_rot * up_face_rot
    return orientation.as_quat()