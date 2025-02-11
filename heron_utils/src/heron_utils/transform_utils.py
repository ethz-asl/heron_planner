#!/usr/bin/env python

import rospy
import numpy as np
import tf_conversions

from scipy.spatial.transform import Rotation

from std_msgs.msg import Time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Transform, TransformStamped
from actionlib_msgs.msg import GoalStatus


def array_from_pose_stamped(pose_stamped: PoseStamped) -> np.array:
    return array_from_pose(pose_stamped.pose)


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


def pose_from_array(
    position: np.ndarray, orientation: np.ndarray = [0, 0, 0, 1]
) -> Pose:

    pose = Pose()
    if len(position) >= 2:
        pose.position.x = position[0]
        pose.position.y = position[1]
        if len(position) == 2:
            pose.position.z = 0.0
        else:
            pose.position.z = position[2]

    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    return pose


def pose_stamped_from_array(
    point: np.ndarray, quat: np.ndarray, frame: str
) -> PoseStamped:

    pose = PoseStamped()
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    if np.shape(point)[0] == 2:
        pose.pose.position.z = 0
    elif np.shape(point)[0] == 3:
        pose.pose.position.z = point[2]
    else:
        raise IndexError

    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time.now()

    return pose

def transform_from_pose_stamped(pose: PoseStamped, child_frame: str) -> TransformStamped:
    """"""
    if isinstance(pose, PoseStamped):
        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = child_frame

        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.x = pose.pose.position.y
        transform.transform.translation.x = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation

        return transform
    else:
        rospy.logerr(f"Pose should be of type PoseStamped: {type(pose)}")
        raise ValueError

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

    # Ensure quaternion has a non-zero norm
    if np.isclose(np.linalg.norm(quaternion), 0):
        raise ValueError("Quaternion has zero norm, cannot calculate angle.")

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


def quaternion_from_angle(
    angle: float, axis: str = "yaw", radians: bool = True
) -> np.ndarray:
    """
    Convert an Euler angle to a quaternion
        - default radiuans, rotation around yaw
        - zyx euler rotation order

    return [x, y, z, w]
    """
    if not radians:
        angle = np.radians(angle)

    if axis == "yaw":
        r = Rotation.from_euler("z", angle)
    elif axis == "pitch":
        r = Rotation.from_euler("y", angle)
    elif axis == "roll":
        r = Rotation.from_euler("x", angle)
    else:
        raise ValueError(f"only [yaw, pitch, roll] accepted, ({axis})")

    return r.as_quat()


def empty_pose() -> Pose:
    """
    empty pose with neutral non-zero quaternion
    """
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.w = 1.0
    return pose


def find_midpoint(point_a: list, point_b: list) -> list:

    midpoint = [
        (point_a[0] + point_b[0]) / 2.0,
        (point_a[1] + point_b[1]) / 2.0,
    ]
    return midpoint


def find_slope(
    point_a: list, point_b: list, perpendicular: bool = False
) -> float:

    if perpendicular == True:
        if point_b[0] - point_a[0] == 0:  # AB is vertical
            m_perp = 0
        elif point_b[1] - point_a[1] == 0:  # AB is horizontal
            m_perp = None
        else:
            m_ab = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])
            m_perp = -1 / m_ab

        return m_perp

    elif perpendicular == False:
        if point_b[0] - point_a[0] == 0:  # AB is vertical
            m_ab = None
        elif point_b[1] - point_a[1] == 0:  # AB is horizontal
            m_ab = 0
        else:
            m_ab = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])

        return m_ab


def find_parallel_pose(
    pose_a: Pose, pose_b: Pose, distance: float, towards: bool = True
) -> Pose:
    """
    finds a new pose
    distance from pose_a along the vector pose_a -> pose_b
    towards: true -> orientation facing pose_a
    """
    [x_a, y_a] = point_from_pose(pose_a)
    [x_b, y_b] = point_from_pose(pose_b)

    vector = np.array([x_b - x_a, y_b - y_a])
    magnitude = np.sqrt(np.sum(vector**2))
    if magnitude == 0:
        raise ValueError("pose_a and pose_b are identical")

    unit_vector = vector / magnitude

    new_point = np.array([x_a, y_a]) + distance * unit_vector
    new_angle = np.arctan2(y_b - y_a, x_b - x_a)

    if towards:
        new_angle = new_angle + np.pi

    new_quat = quaternion_from_angle(new_angle)

    return pose_from_point(new_point, new_quat)


def find_perpendicular_pose(
    pose_a: Pose, pose_b: Pose, distance: float, towards: bool = True
) -> Pose:
    """
    finds a new pose perpendicular
    distance from pose_a along the vector pose_a -> pose_b
    towards: true -> orientation facing pose_a
    """
    [x_a, y_a] = point_from_pose(pose_a)
    [x_b, y_b] = point_from_pose(pose_b)

    vector = np.array([x_b - x_a, y_b - y_a])
    magnitude = np.sqrt(np.sum(vector**2))
    if magnitude == 0:
        raise ValueError("pose_a and pose_b are identical")

    unit_vector = vector / magnitude
    perp_vector = np.array([-unit_vector[1], unit_vector[0]])  # ccw

    if not towards:
        perp_vector = -perp_vector  # cw

    new_point = np.array([x_a, y_a]) + distance * perp_vector
    new_angle = np.arctan2(perp_vector[1], perp_vector[0])

    new_quat = quaternion_from_angle(new_angle)

    return pose_from_point(new_point, new_quat)


def find_offset_from_pose(
    pose: PoseStamped, offset_dist: float, towards: bool = True
) -> PoseStamped:
    """
    calculate offset pose facing towards the pose

    towards -> facing towards or parallel?
    """

    pose_arr = array_from_pose(pose.pose)
    pos = pose_arr[:3]

    # position
    dir = pos / np.linalg.norm(pos)
    offset_pos = pos + dir * offset_dist  # dir vector from frame origin -> pose

    offset_pose = PoseStamped()
    offset_pose.header = pose.header
    offset_pose.pose.position.x = offset_pos[0]
    offset_pose.pose.position.y = offset_pos[1]
    offset_pose.pose.position.z = offset_pos[2]

    # orientation
    if towards:  # facing pose
        yaw = np.arctan2(dir[1], dir[0])  # xy plane
    else:  # parallel to pose
        yaw = np.arctan2(dir[0], -dir[1])

    quat = quaternion_from_angle(yaw)
    offset_pose.pose.orientation = Quaternion(*quat)

    return offset_pose


def pose_from_point(point: list, orientation: list = [0, 0, 0, 1]) -> Pose:
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    if len(point) > 2:
        pose.position.z = point[2]

    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    return pose


def point_from_pose(pose: Pose) -> list:
    return [pose.position.x, pose.position.y]


def quaternion_from_pose(pose: Pose) -> list:
    return [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


def at_pose(pose1: Pose, pose2: Pose, tol: float = 0.01) -> bool:
    disp_xy, disp_th = displacement_from_pose(pose1, pose2)
    return abs(disp_xy) < tol and abs(disp_th) < tol


def displacement_from_pose(
    pose1: Pose, pose2: Pose, printing: bool = False
) -> list:
    """Get distance between target and current pose [m, deg]"""
    if isinstance(pose1, Pose):
        pose1_arr = array_from_pose(pose1)

    pose1_xy = pose1_arr[:2]
    pose1_th = angle_from_quaternion(pose1_arr[3:], "yaw")
    pose1_th = wrap_angle(pose1_th)

    if isinstance(pose2, Pose):
        pose2_arr = array_from_pose(pose2)

    pose2_xy = pose2_arr[:2]
    pose2_th = angle_from_quaternion(pose2_arr[3:], "yaw")
    pose2_th = wrap_angle(pose2_th)

    disp_xy = round(np.linalg.norm(pose2_xy - pose1_xy) - 0.5, 3)
    delta_th = pose1_th - pose2_th

    disp_th = round(delta_th, 3)
    disp_th = wrap_angle(disp_th)  # wrapping around [-180,180]

    if printing:
        rospy.logwarn(
            f"{abs(disp_xy):.2f}[m] and {abs(disp_th):.2f}[rad] from target"
        )

    return [disp_xy, disp_th]


def at_gps(
    gps1: NavSatFix,
    gps2: NavSatFix,
    tol: float = 0.01,
    check_altitude: bool = False,
) -> bool:
    """calcs if gps coords are close enough
    tol in meters"""
    lat_lon_dist = haversine_dist(gps1, gps2)

    # alt diff
    alt_diff = 0.0
    if check_altitude:
        if (
            gps1.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
            and gps2.position_covariance_type
            != NavSatFix.COVARIANCE_TYPE_UNKNOWN
        ):
            alt_diff = abs(gps2.altitude - gps1.altitude)

    return lat_lon_dist < tol and alt_diff < tol


def haversine_dist(gps1: NavSatFix, gps2: NavSatFix) -> float:
    """calc distance in m from 2 gps coordinates"""
    R = 6371000  # earth's radius in m

    lat1 = np.deg2rad(gps1.latitude)
    lat2 = np.deg2rad(gps2.latitude)

    dlat = np.deg2rad(gps2.latitude - gps1.latitude)
    dlon = np.deg2rad(gps2.longitude - gps1.longitude)

    a = (
        np.sin(dlat / 2.0) ** 2
        + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    )
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return R * c
