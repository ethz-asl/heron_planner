#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from heron_msgs.srv import FindCrack, FindCrackRequest, FindCrackResponse, \
    FindLaneEdge, FindLaneEdgeRequest, FindLaneEdgeResponse, FindPothole, \
    FindPotholeRequest, FindPotholeResponse
from heron_utils.async_service_proxy import AsyncServiceProxy

class FindCrackBehaviour(pt.behaviour.Behaviour):
    """
    Communicate with the ICCS detection interface to find potholes.
    Do nothing if the potholes are too small.
    """

    def __init__(self, name: str):
        super().__init__(name)

    def setup(self):
        #self._min_pothole_size_m = rospy.get_param(
        #    '~/' + name + '/min_pothole_size_m', 0.0)
        # TODO: decide if these are "hard-coded" (but can be remapped in a
        # launch file) or declared in the yaml.
        self._find_crack_service_name = "/iccs_crack_finder/find_cracks"
        self._rgb_image_topic_name = "/front_rgbd_camera/rgb/image_raw"
        self._depth_image_topic_name = "/front_rgbd_camera/stereo/image"
        self._camera_info_topic_name = "/front_rgbd_camera/stereo/camera_info"

        self._service_client = AsyncServiceProxy(
            self._find_crack_service_name, FindCrack)

        self._rgb_sub = rospy.Subscriber(
            self._rgb_image_topic_name, Image, self.rgb_image_callback)
        self._depth_sub = rospy.Subscriber(
            self._depth_image_topic_name, Image, self.depth_image_callback)
        self._camera_info_sub = rospy.Subscriber(
            self._camera_info_topic_name, CameraInfo, self.camera_info_callback)

        # Cache the messages
        self._rgb_image_msg = None
        self._depth_image_msg = None
        self._camera_info_msg = None
        self._service_future = None
        self._waiting_for_images = False

    def initialise(self):
        # Reset messages
        self._rgb_image_msg = None
        self._depth_image_msg = None
        self._camera_info_msg = None
        self._service_future = None
        self._waiting_for_images = True

    def update(self) -> pt.common.Status:
        if self._waiting_for_images:
            # Check if we have all the messages:
            if self._rgb_image_msg and self._depth_image_msg and self._camera_info_msg:
                self._waiting_for_images = False
                rospy.loginfo("Calling service call.")

                service_request = FindCrackRequest()
                service_request.image_rgb = self._rgb_image_msg
                service_request.image_depth = self._depth_image_msg
                service_request.camera_info = self._camera_info_msg
                try:
                    self._service_future = self._service_client(service_request)
                except rospy.service.ServiceException as exc:
                    rospy.loginfo("Service did not process request: " + str(exc))
                    return pt.common.Status.FAILURE
            else:
                rospy.loginfo("Waiting for images.")
                return pt.common.Status.RUNNING
        else:
            # Check if the future is returned yet
            if self._service_future.done():
                # The result is here, do something with it.
                print(self._service_future.result())
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.RUNNING
        return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        self._rgb_image_msg = None
        self._depth_image_msg = None
        self._camera_info_msg = None
        self._service_future = None
        self._waiting_for_images = False

    def rgb_image_callback(self, image_msg):
        if self._waiting_for_images:
            rospy.loginfo("Got an RGB image.")
            self._rgb_image_msg = image_msg

    def depth_image_callback(self, image_msg):
        if self._waiting_for_images:
            rospy.loginfo("Got an Depth image.")
            self._depth_image_msg = image_msg

    def camera_info_callback(self, camera_info_msg):
        if self._waiting_for_images:
            rospy.loginfo("Got an Camera Info image.")
            self._camera_info_msg = camera_info_msg

def main():
    rospy.init_node("vision_behaviors")

    pt.logging.level = pt.logging.Level.DEBUG
    crack_behavior = FindCrackBehaviour("find_crack")
    crack_behavior.setup()
    while not rospy.is_shutdown():
        crack_behavior.tick_once()
        rospy.sleep(0.05)
    crack_behavior.stop(pt.common.Status.INVALID)

if __name__ == "__main__":
    main()
