#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import threading
import numpy as np

from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from heron_msgs.srv import (
    FindPothole,
    FindPotholeRequest,
    FindPotholeResponse,
    FindCrack,
    FindLaneEdge,
)

# can't have same class name as service call
class SearchForPothole(pt.behaviour.Behaviour):
    """need to send this before sending a new cmd, new cmds do not overwrite old"""

    def __init__(
        self, name: str, img_rgb: Image, img_depth: Image, cam_info: CameraInfo
    ) -> None:
        super().__init__(name)
        self._img_rgb = img_rgb
        self._img_depth = img_depth
        self._cam_info = cam_info
        self._thread = None

        self._center_of_mass_pub = rospy.Publisher(
            "pothole/center_of_mass", PoseStamped, queue_size=10
        )
        self._surface_area_pub = rospy.Publisher(
            "pothole/surface_area", Float64, queue_size=10
        )

        self._srv_name = "/iccs/find_pothole"
        self._srv_setup = None
        self._res_recieved = None
        self._req = FindPotholeRequest()
        self._res = FindPotholeResponse()

    def setup(self) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, FindPothole)
            rospy.wait_for_service(self._srv_name, timeout=rospy.Duration(2.0))
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Service {self._srv_name} setup failed: {err}")

    def initialise(self):
        self._srv_setup = False
        self._res_recieved = False
        self._srv_setup = self.setup()

        if not self._srv_setup:
            rospy.logwarn(f"Service {self._srv_name} failed")
            return

        self._req.image_rgb = self._img_rgb
        self._req.image_depth = self._img_depth
        self._req.camera_info = self._cam_info

        self._thread = threading.Thread(target=self._call_srv)
        self._thread.start()

    def _call_srv(self):
        rospy.logerr("entered call_srv")
        try:
            rospy.logerr("Calling FindPothole srv...")
            self._res = self._client(self._req)
            self._res_recieved = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service call failed: {err}")
            self._res = None
            self._res_recieved = False

    def update(self) -> pt.common.Status:
        rospy.logerr(f"has res been recieved? {self._res_recieved}")

        if not self._res_recieved:
            return pt.common.Status.RUNNING

        if self._res and self._res.success:
            rospy.loginfo("Success, pothole found")
            self._center_of_mass_pub.publish(self._res.center_of_mass)
            self._surface_area_pub.publish(Float64(data=self._res.surface_area_m))
            return pt.common.Status.SUCCESS
        
        rospy.loginfo(
            f"Failed, pothole not found ({self._res.success}) or srv call failed"
        )
        return pt.common.Status.FAILURE

    def terminate(self, new_status):
        if self._thread and self._thread.is_alive():
            self._thread.join()

class SendPhotoToKafka(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

    def update(self):
        # Mock handle detection logic
        print("Handling detection: Sending CoM and photo to Kafka.")
        return pt.common.Status.SUCCESS
