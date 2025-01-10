#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

from std_msgs.msg import Empty
from heron_msgs.srv import (
    FindPothole,
    FindCrack,
    FindLaneEdge
)

class FindPothole(pt.behaviour.Behaviour):
    """need to send this before sending a new cmd, new cmds do not overwrite old"""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._srv_name = "iccs/find_pothole"
        self._req = FindPothole()

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, FindPothole)
            rospy.wait_for_service(self._srv_name, timeout=timeout)
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Service {self._srv_name} setup failed: {err}")

    def initialise(self):
        self._srv_setup = False
        self._srv_called = False
        self._res = None

        self._srv_setup = self.setup()

        if not self._srv_setup:
            rospy.logwarn(f"Service {self._srv_name} failed")
            return

        try:
            rospy.loginfo(f"Sending command {self._req}")
            self._res = self._client(self._req)
            self._srv_called = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:

        if not self._srv_setup or not self._srv_called:
            rospy.loginfo(f"Failed: {self._srv_setup} and {self._srv_called}")
            return pt.common.Status.FAILURE
        if self._res and self._res.ret.success:
            rospy.loginfo("Success")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
