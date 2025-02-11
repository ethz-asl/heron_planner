#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from heron_msgs.srv import (
    TransformPose,
    TransformPoseRequest,
    TransformPoseResponse,
    GetSynchedImages,
    GetSynchedImagesRequest,
    GetSynchedImagesResponse,
)
import threading

import tf2_ros
import tf2_geometry_msgs


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

        # advertise service
        self.service = rospy.Service(
            "hlp/get_synched_images", GetSynchedImages, self.handle_sync_req
        )
        rospy.loginfo("Image data service ready.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.transform_srv = rospy.Service(
            "hlp/transform_pose", TransformPose, self.handle_transform_req
        )
        rospy.loginfo("transform pose service is ready.")


    def handle_transform_req(self, req: TransformPoseRequest):
        try:
            target_frame = req.target_frame
            pose_in = req.pose_in
            rospy.loginfo(
                f"current frame: {pose_in.header.frame_id}, target frame: {target_frame}"
            )

            # lookup transform from the input pose frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_in.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),  # wait time for the transform
            )

            pose_out = tf2_geometry_msgs.do_transform_pose(pose_in, transform)
            pose_out.header.stamp = pose_in.header.stamp  # syncing timestamp

            return TransformPoseResponse(success=True, pose_out=pose_out)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(f"Transform error: {str(e)}")
            return TransformPoseResponse(
                success=False,
            )

    def handle_sync_req(self, req: GetSynchedImagesRequest) -> GetSynchedImagesResponse:
        camera_ns = req.camera_ns.rstrip("/")

        # topic names
        rgb_topic = f"{camera_ns}/rgb/image_raw"
        depth_topic = f"{camera_ns}/depth/image_raw"
        info_topic = f"{camera_ns}/color/camera_info"

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
