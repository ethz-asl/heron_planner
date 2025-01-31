#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image, CameraInfo
from heron_msgs.srv import (
    GetSynchedImages,
    GetSynchedImagesRequest,
    GetSynchedImagesResponse,
)
from threading import Event


class SynchedImagesService:
    def __init__(self) -> None:
        rospy.init_node("get_synched_images_server")

        # dict to store latest msgs
        self.latest_rgb = {}
        self.latest_depth = {}
        self.latest_info = {}

        # synchronisation events to wait for new msgs
        self.rgb_event = Event()
        self.depth_event = Event()
        self.info_event = Event()

        # advertise service
        self.service = rospy.Service(
            "hlp/get_synched_images", GetSynchedImages, self.handle_req
        )
        rospy.loginfo("Image data service ready.")

    def handle_req(self, req: GetSynchedImagesRequest) -> GetSynchedImagesResponse:
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
            self.rgb_event.wait(2.0)
            and self.depth_event.wait(2.0)
            and self.info_event.wait(2.0)
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
    SynchedImagesService()
    rospy.spin()
