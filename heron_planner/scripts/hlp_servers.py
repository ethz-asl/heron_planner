#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
from scipy.spatial.distance import cdist
from scipy.ndimage import convolve1d
import scipy.interpolate as si
from sklearn.linear_model import RANSACRegressor
import numpy as np

from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path

import heron_utils.transform_utils as utils
from heron_msgs.srv import (
    TransformPose,
    TransformPoseRequest,
    TransformPoseResponse,
    GetSynchedImages,
    GetSynchedImagesRequest,
    GetSynchedImagesResponse,
    FindCrackPath,
    FindCrackPathRequest,
    FindCrackPathResponse,
    FindOffset,
    FindOffsetRequest,
    FindOffsetResponse,
)
import threading

import tf2_ros
import tf2_geometry_msgs


POTHOLE_OFFSET = rospy.get_param("/pothole/offset", 0.5)
CRACK_OFFSET = rospy.get_param("/cracks/offset", 0.7)
CRACK_SIDE_RIGHT = rospy.get_param("/cracks/is_left_side", True)
CONE_OFFSET = rospy.get_param("/cone_place/offset", 0.7)
CONE_SIDE_LEFT = rospy.get_param("/cone_place/is_left", True)
CAM_FRAME = rospy.get_param("/ugv/arm_cam_frame", "front_rgbd_camera_rgb_camera_optical_frame")
BASE_FRAME = rospy.get_param("/ugv/base_frame", "robot_base_footprint")
MAP_FRAME = rospy.get_param("/ugv/map_frame", "robot_map")
MAX_CRACK_LENGTH = rospy.get_param("/cracks/max_length")

class HLPServers:
    # class TransformFinder:
    def __init__(self):
        rospy.init_node("hlp_services")

        self.timeout = 5.0

        # dict to store latest msgs
        self.latest_rgb = {}
        self.latest_depth = {}
        self.latest_info = {}

        # synchronisation events to wait for new msgs
        self.rgb_event = threading.Event()
        self.depth_event = threading.Event()
        self.info_event = threading.Event()

        # for rqt
        self.path_pub = rospy.Publisher("hlp/crack_path", Path, queue_size=1)
        self.path_overlay = rospy.Publisher(
            "hlp/crack_path_overlay", Image, queue_size=1
        )
        self.path_start_pub = rospy.Publisher("hlp/crack_path_start", PoseStamped, queue_size=5)
        self.path_end_pub = rospy.Publisher("hlp/crack_path_end", PoseStamped, queue_size=5)

        # advertise service
        self.service = rospy.Service(
            "hlp/get_synched_images", GetSynchedImages, self.handle_sync_req
        )
        rospy.loginfo("Image data service ready.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.transform_srv = rospy.Service(
            "hlp/transform_pose", TransformPose, self.handle_transform_req
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_tf)
        self.latest_defect = None
        self.latest_offset = None
        rospy.loginfo("transform pose service is ready.")

        self.offset_srv = rospy.Service(
            "hlp/find_offset",  FindOffset, self.handle_offset_req
        )

        self.bridge = cv_bridge.CvBridge()

        self.crack_path_srv = rospy.Service(
            "hlp/generate_crack_path", FindCrackPath, self.handle_crack_req
        )
        self.contour_thresh = rospy.get_param("/cracks/contour_threshold", 80)

        rospy.loginfo("generating crack path service is ready")

    def transform_point(
        self, point: PointStamped, target_frame: str = "odom"
    ) -> PointStamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            return tf2_geometry_msgs.do_transform_point(point, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(f"TF Transform error: {err}")
            return None

    def filter_outliers(self, path_points, window_size=10, deviation_thresh=5):
        """removes outliers in path based on col-wide averaging"""

        # compute col wise filtering
        filtered_points = []
        for idx in range(0, len(path_points), window_size // 2): # overlapping
            window = path_points[idx:idx + window_size]

            if len(window) < 3:
                filtered_points.extend(window.tolist())
                continue

            # fit line usiong RANSAC
            x = window[:, 0].reshape(-1, 1) # col
            y = window[:, 1] # row

            model = RANSACRegressor()
            model.fit(x, y)
            y_pred = model.predict(x)

            # filter points
            for jdx, (col, row) in enumerate(window):
                if abs(row - y_pred[jdx]) <= deviation_thresh: # keep close points
                    filtered_points.append((col, row)) 

        return np.array(filtered_points)

    def extract_col(self, mask, window_size=10, deviation_thresh=5):
        """extract path from avg col segment"""
        height, width = mask.shape
        path_points = []

        avg_locs = np.full(width, np.nan)  # store column-wise avg

        for col in range(width):
            column = mask[:, col]

            # 1d convolution
            smoothed = convolve1d(column, weights=np.ones(window_size)/window_size, mode='constant')
            if np.sum(smoothed) > 0: # if segment is in column
                avg_v = np.sum(np.arange(height) * smoothed) / np.sum(smoothed)
                path_points.append((col, int(avg_v)))  # (u, v) pixel

        return np.array(path_points)

    def smooth_path(self, path_points, smoothing_factor=0.1, num_samples=100):
        """smooth path using b-spline filter"""

        x, y = path_points[:, 0], path_points[:, 1]

        # create b-spline representation
        tck, u = si.splprep([x, y], s=smoothing_factor)

        # sample new smooth points alone spline
        u_fine = np.linspace(0, 1, num_samples)
        x_smooth, y_smooth = si.splev(u_fine, tck)
        return np.column_stack((x_smooth, y_smooth))

    def trim_path(self, path_points, trim_length: float = 0.6, scale: float = 0.001):
        """
        trims the path to ensure its length does not exceed by keeping the middle 60cm segment.

        :param path_points: (N, 2) or (N, 3) NumPy array representing path points
        :param scale: Scale factor to convert points to meters
        :return: trimmed path as a NumPy array
        """
        # compute cumulative distances along the path
        distances = np.cumsum(np.linalg.norm(np.diff(path_points, axis=0), axis=1))
        distances = np.insert(distances, 0, 0)  # start distance is zero

        total_length = distances[-1] * scale
        if total_length <= trim_length:
            return path_points  # no need to trim

        # Find the middle point
        middle_idx = np.searchsorted(distances, distances[-1] / 2)

        # Find the indices for ±trim_length/2 cm range
        lower_bound = np.searchsorted(distances, distances[middle_idx] - (trim_length/2) / scale)
        upper_bound = np.searchsorted(distances, distances[middle_idx] + (trim_length/2) / scale)

        # Slice the path within the desired range
        return path_points[lower_bound:upper_bound + 1]

    def handle_crack_req(self, req: FindCrackPathRequest):

        # filter to just large segments
        mask = self.bridge.imgmsg_to_cv2(
            req.segmentation_mask, desired_encoding="mono8"
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        filtered_mask = np.zeros_like(mask)

        for contour in contours:
            if cv2.contourArea(contour) > self.contour_thresh:
                cv2.drawContours(
                    filtered_mask, [contour], -1, (255), thickness=cv2.FILLED
                )

        # create mask to publish to ros
        overlay_mask = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)

        # extract columns
        path_points = self.extract_col(filtered_mask)

        # path_points = self.filter_outliers(path_points)

        # smooth path
        smooth_points = self.smooth_path(path_points)

        scale = 0.001  # in m
        dist = np.linalg.norm(smooth_points[-1] - smooth_points[0]) * scale
        if dist > MAX_CRACK_LENGTH:
            smooth_points = self.trim_path(smooth_points, MAX_CRACK_LENGTH)


        for idx in range(len(smooth_points) - 1):
            cv2.line(
                overlay_mask,
                tuple(map(int, smooth_points[idx])),
                tuple(map(int, smooth_points[idx + 1])),
                (255, 0, 255),
                2,
            )

        path = Path()
        path.header.stamp = rospy.Time.now()
        # path.header.frame_id = MAP_FRAME
        path.header.frame_id = BASE_FRAME

        for x, y in smooth_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.header.frame_id = BASE_FRAME
            pose.pose.position.x = x * scale
            pose.pose.position.y = y * scale
            pose.pose.position.z = 0  # road assumed 2d or add offset here TODO
            pose.pose.orientation.w = 1 # assume pose points up (can change later) TODO

            # Transform pose to robot_map or world
            transformed_pose = self.transform_pose(pose, BASE_FRAME)  # or "world"
            if transformed_pose:
                transformed_pose.pose.position.z = 0  # road assumed 2d or add offset here TODO
                transformed_pose.pose.orientation.w = 1 # assume pose points up (can change later) TODO
                path.poses.append(transformed_pose)
            else:
                rospy.logwarn("Failed to transform crack path pose")


        rospy.logwarn(f"Path distance = {dist:.2f}")

        #TODO if path is longer than robot workspace

        start = path.poses[0]
        rospy.logwarn(f"start {start}")
        end = path.poses[-1]
        rospy.logwarn(f"end {end}")
        mid = path.poses[round(len(path.poses) / 2)]
        #TODO should find middle between start & end here
        #TODO also publish beginning & end
        rospy.logwarn(f"mid {mid}")
        
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay_mask, encoding="bgr8")
        overlay_msg.header.stamp = rospy.Time.now()
        self.path_overlay.publish(overlay_msg)
        rospy.logwarn(f"published overlay path")
   
        self.path_pub.publish(path)
        self.path_start_pub.publish(start)
        self.path_end_pub.publish(end)
        rospy.loginfo("Published crack path")
        # rospy.logwarn(f"PATH: {path}")
        return FindCrackPathResponse(success=True, path=path, middle_pose=mid)

    def transform_pose(self, pose: PoseStamped, target_frame: str = "odom") -> PoseStamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # Target frame (e.g., "odom" or "robot_map")
                pose.header.frame_id,  # Source frame ("camera_link")
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            return tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(f"TF Transform error: {err}")
            return None


    def handle_transform_req(self, req: TransformPoseRequest):
        try:
            pose_out = self.transform_pose(req.pose_in, req.target_frame)
            return TransformPoseResponse(success=True, pose_out=pose_out)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(f"Transform error: {str(e)}")
            return TransformPoseResponse(success=False)

    def handle_offset_req(
            self, req: FindOffsetRequest
    ) -> FindOffsetResponse:

        res = FindOffsetResponse()
        res.success = True

        pose_arr = utils.array_from_pose(req.defect_pose.pose)
        x = pose_arr[0]
        y = pose_arr[1]
        yaw = utils.angle_from_quaternion(pose_arr[3:])
        rospy.logwarn(f"defect pose: {req.defect_pose}")

        if req.defect_type == "pothole":
            rospy.logwarn(f"Offset: {POTHOLE_OFFSET}")
            x += POTHOLE_OFFSET * np.cos(yaw)
            y += -POTHOLE_OFFSET * np.sin(yaw)
            new_yaw = yaw + np.pi  # rotate to face pothole

        elif req.defect_type == "crack":
            if CRACK_SIDE_RIGHT:
                x += CRACK_OFFSET * np.cos(yaw - np.pi/2)
                y += CRACK_OFFSET * np.sin(yaw - np.pi/2)
            else:
                x += CRACK_OFFSET * np.cos(yaw + np.pi/2)
                y += CRACK_OFFSET * np.sin(yaw + np.pi/2)
            new_yaw = yaw  # no rotation
        elif req.defect_type == "cones":
            if CONE_SIDE_LEFT:
                x += CONE_OFFSET * np.cos(yaw + np.pi/2)
                y += CONE_OFFSET * np.sin(yaw + np.pi/2)
            else:
                x += CONE_OFFSET * np.cos(yaw - np.pi/2)
                y += CONE_OFFSET * np.sin(yaw - np.pi/2)

            new_yaw = yaw  # no rotation

        quat = utils.quaternion_from_angle(new_yaw)

        res.offset_pose = PoseStamped()
        res.offset_pose.header = req.defect_pose.header  # Keep same frame
        
        res.offset_pose.pose.position.x = x
        res.offset_pose.pose.position.y = y
        res.offset_pose.pose.position.z = req.defect_pose.pose.position.z
        
        res.offset_pose.pose.orientation.x = quat[0]
        res.offset_pose.pose.orientation.y = quat[1]
        res.offset_pose.pose.orientation.z = quat[2]
        res.offset_pose.pose.orientation.w = quat[3]        
        rospy.logwarn(f"offset pose: {res.offset_pose}")
        rospy.logwarn(f"defect pose: {req.defect_pose}")

        # broadcast to TF
        if req.broadcast_to_tf:
            self.latest_defect = self.broadcast_offset_tf(req.defect_pose, req.defect_type)
            self.latest_offset = self.broadcast_offset_tf(res.offset_pose, req.broadcast_frame)

        return res
        

    def broadcast_offset_tf(self, pose: PoseStamped, frame: str) -> TransformStamped:
        return utils.transform_from_pose_stamped(pose, child_frame=frame)
        
    
    def publish_tf(self, event):

        if self.latest_defect:
            self.latest_defect.header.stamp = rospy.Time.now()
            self.latest_defect.transform.translation.z = 0
            self.tf_broadcaster.sendTransform(self.latest_defect)

        if self.latest_offset:
            self.latest_offset.header.stamp = rospy.Time.now()
            self.latest_offset.transform.translation.z = 0
            self.tf_broadcaster.sendTransform(self.latest_offset)

    def handle_sync_req(
        self, req: GetSynchedImagesRequest
    ) -> GetSynchedImagesResponse:
        camera_ns = req.camera_ns.rstrip("/")

        # topic names
        rgb_topic = f"{camera_ns}/rgb/image_raw"
        depth_topic = f"{camera_ns}/stereo/image_raw"
        info_topic = f"{camera_ns}/rgb/camera_info"

        rospy.loginfo(
            f"Fetching data from: {rgb_topic}, {depth_topic}, {info_topic}"
        )

        # sub to topics
        self.rgb_event.clear()
        self.depth_event.clear()
        self.info_event.clear()

        rgb_sub = rospy.Subscriber(
            rgb_topic, Image, self.rgb_cb, callback_args=camera_ns
        )
        depth_sub = rospy.Subscriber(
            depth_topic, Image, self.depth_cb, callback_args=camera_ns
        )
        info_sub = rospy.Subscriber(
            info_topic, CameraInfo, self.info_cb, callback_args=camera_ns
        )

        # wait for data w/ timeout
        success = (
            self.rgb_event.wait(self.timeout)
            and self.depth_event.wait(self.timeout)
            and self.info_event.wait(self.timeout)
        )

        # unsub to topics
        rgb_sub.unregister()
        depth_sub.unregister()
        info_sub.unregister()

        if (
            success
            and camera_ns in self.latest_rgb
            and camera_ns in self.latest_depth
            and camera_ns in self.latest_info
        ):
            rospy.loginfo(f"successfully retrieved image data")
            return GetSynchedImagesResponse(
                image_rgb=self.latest_rgb[camera_ns],
                image_depth=self.latest_depth[camera_ns],
                camera_info=self.latest_info[camera_ns],
                success=True,
            )
        else:
            rospy.logwarn(f"failed to get camera data within timeout")
            return GetSynchedImagesResponse(success=False)

    def rgb_cb(self, msg: Image, camera_ns: str) -> None:
        self.latest_rgb[camera_ns] = msg
        self.rgb_event.set()

    def depth_cb(self, msg: Image, camera_ns: str) -> None:
        self.latest_depth[camera_ns] = msg
        self.depth_event.set()

    def info_cb(self, msg: Image, camera_ns: str) -> None:
        self.latest_info[camera_ns] = msg
        self.info_event.set()


if __name__ == "__main__":
    try:
        HLPServers()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
