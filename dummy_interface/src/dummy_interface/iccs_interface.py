#!/usr/bin/env python

from __future__ import annotations  # for type hinting
from typing import Callable

import rospy
import rospkg
import numpy as np
import cv_bridge
import random

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo

from heron_msgs.srv import (
    FindPothole,
    FindPotholeRequest,
    FindPotholeResponse,
    FindCrack,
    FindCrackRequest,
    FindCrackResponse,
    FindLaneEdge,
    FindLaneEdgeRequest,
    FindLaneEdgeResponse,
)

"""
Add here all the items for making a interface to ICCS

E.G.
Sequences
Commands
as SRV requests!
publish status to kafka...

"""


class ICCSDummyInterface:
    """ """

    def __init__(self) -> None:
        rospy.init_node("dummy_iccs_interface")

        self.cv_bridge = cv_bridge.CvBridge()

        # services
        self.find_pothole_srv = rospy.Service(
            "/iccs/find_pothole",
            FindPothole,
            self.handle_find_pothole,
        )

        self.find_crack_srv = rospy.Service(
            "/iccs/find_crack",
            FindCrack,
            self.handle_find_crack,
        )

        self.find_line_edge_srv = rospy.Service(
            "/iccs/find_lane_edge",
            FindLaneEdge,
            self.handle_find_lane_edge,
        )

        rospy.loginfo("Dummy ICCS interface services are ready.")

    def handle_find_pothole(
        self, req: FindPotholeRequest
    ) -> FindPotholeResponse:
        """"""
        cv_img = self.cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding="passthrough"
        )
        rospy.loginfo(cv_img.shape)

        success = True

        center_of_mass = PoseStamped()
        center_of_mass.pose.position.x = 0
        center_of_mass.pose.position.y = 0
        center_of_mass.pose.position.z = 0
        center_of_mass.pose.orientation.w = 0

        surface_area = random.uniform(0.08, 0.2)

        return self.generate_pothole_res(success, center_of_mass, surface_area)

    def handle_find_crack(self, req: FindCrackRequest) -> FindCrackResponse:
        """"""
        cv_img = self.cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding="passthrough"
        )
        rospy.loginfo(cv_img.shape)

        success = True
        
        start_point = PointStamped()
        start_point.point.x = 0
        start_point.point.y = 0
        start_point.point.z = 0
        
        end_point = PointStamped()
        end_point.point.x = 1.0
        end_point.point.y = 1.0
        end_point.point.z = 0

        return self.generate_crack_res(success, start_point, end_point)

    def handle_find_lane_edge(
        self, req: FindLaneEdgeRequest
    ) -> FindLaneEdgeResponse:
        """"""
        cv_img = self.cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding="passthrough"
        )
        rospy.loginfo(cv_img.shape)

        success = True
        
        start_point = PointStamped()
        start_point.point.x = 0
        start_point.point.y = 0
        start_point.point.z = 0
        
        end_point = PointStamped()
        end_point.point.x = 1.0
        end_point.point.y = 1.0
        end_point.point.z = 0

        return self.generate_lane_edge_res(success, start_point, end_point)


    def generate_pothole_res(
        self, success: bool, center_of_mass: PoseStamped, surface_area: float
    ) -> FindPotholeResponse:
        """utility function for standard mode reponse"""
        res = FindPotholeResponse()
        res.success = success
        res.center_of_mass = center_of_mass
        res.surface_area_m = surface_area
        return res

    def generate_crack_res(
        self, success: bool, start_point: PointStamped, end_point: PointStamped
    ) -> FindCrackResponse:
        """utility function for standard mode reponse"""
        res = FindCrackResponse()
        res.success = success
        res.start_point = start_point
        res.end_point = end_point
        return res

    def generate_lane_edge_res(
        self, success: bool, start_point: PointStamped, end_point: PointStamped
    ) -> FindLaneEdgeResponse:
        """utility function for standard mode reponse"""
        res = FindLaneEdgeResponse()
        res.success = success
        res.start_point = start_point
        res.end_point = end_point
        return res

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    try:
        ugv_interface = ICCSDummyInterface()
        ugv_interface.run()

    except rospy.ROSInterruptException as err:
        rospy.logwarn(f"Shutting down interface : {err}")
