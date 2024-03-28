#!/usr/bin/env python

import rospy
import numpy as np

#  import scipy as sc
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

def array_from_pose(pose: Pose) -> np.array:
    array = np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    return array


def wrap_angle(
    angle: float, min_angle: float = -np.pi, max_angle: float = np.pi
) -> float:
    """
    Wrap an angle around limits
        - default [-pi, pi] 
        - angle unit should match limit 
    """
    range_width = max_angle - min_angle
    return np.mod(angle + max_angle, range_width) + min_angle


def angle_from_quaternion(
    quaternion: np.ndarray, axis: str = "yaw", radians: bool = True
) -> float:
    """
    Convert a quaternion to an Euler angle
        - default radians, rotation around yaw 
        - zyx euler rotation order
    """

    r = Rotation.from_quat(quaternion)

    if axis == "yaw":
        r = r.as_euler("zyx")[0]
    elif axis == "pitch":
        r = r.as_euler("zyx")[1]
    elif axis == "roll":
        r = r.as_euler("zyx")[2]
    else:
        raise ValueError(
            f"Only [yaw, pitch, roll] are accepted (yours: [{axis}])"
        )

    if radians:
        return r
    else:
        return np.degree(r)
