#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

import heron_utils.transform_utils as utils

from sensor_msgs.msg import NavSatFix

from robot_simple_command_manager_msgs.srv import SetCommandString
from heron_msgs.srv import (
    ChangeRobotMode,
    ChangeRobotModeRequest,
    ChangeRobotModeResponse,
)


class ROSWait(pt.behaviour.Behaviour):
    def __init__(self, name: str, wait_time: float) -> None:
        super().__init__(name)
        self._wait_time = wait_time

    def initialise(self):
        self._rate = rospy.Rate(1.0 / self._wait_time)

    def update(self) -> pt.common.Status:
        rospy.loginfo(f"Sleeping for {self._wait_time} seconds ...")
        self._rate.sleep()
        return pt.common.Status.SUCCESS


class Wait(pt.behaviour.Behaviour):
    def __init__(self, name: str, wait_time: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_manager/command"
        self._req = "WAIT " + wait_time

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, SetCommandString)
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


class Move(pt.behaviour.Behaviour):
    def __init__(self, name: str, x_dir: str, y_dir: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_manager/command"
        self._req = "MOVE " + x_dir + " " + y_dir

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, SetCommandString)
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


class Turn(pt.behaviour.Behaviour):
    def __init__(self, name: str, angle: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_manager/command"
        self._req = "TURN " + angle

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, SetCommandString)
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


class AtGPS(pt.behaviour.Behaviour):
    def __init__(self, name: str, goal_gps: NavSatFix) -> None:
        super().__init__(name)
        self.goal_gps = goal_gps
        self.current_gps = None

        self.recieved_gps = False
        self.at_gps = None

        rospy.Subscriber("/robot/gps/fix", NavSatFix, self.gps_cb)

    def gps_cb(self, msg: NavSatFix) -> None:        
        self.current_gps = msg
        self.recieved_gps = True

    def initialise(self):
        if self.recieved_gps is True:
            self.at_gps = utils.at_gps(self.current_gps, self.goal_gps) 

    def update(self) -> pt.common.Status:

        if not self.at_gps:
            rospy.loginfo(f"Failed: not at GPS location")
            return pt.common.Status.FAILURE
        elif self.at_gps:
            rospy.loginfo("Success, at GPS location")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class GoToGPS(pt.behaviour.Behaviour):
    def __init__(self, name: str, gps: NavSatFix):
        super().__init__(name)
        self._srv_name = "/robot/command_manager/command"
        #TODO define this w/ Raquel
        yaw = str(0.0)
        self._req = "GOTO_GPS " + str(gps.latitude) + " " + str(gps.longitude) + " " + yaw

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, SetCommandString)
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


class Dock(pt.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_manager/command"
        self._req = "DOCK"

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, SetCommandString)
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


class ChangeRobotMode(pt.behaviour.Behaviour):
    def __init__(self, name: str, change_req: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/change_mode"
        self._req = ChangeRobotModeRequest(mode=change_req)
        self._res = ChangeRobotModeResponse()

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, ChangeRobotMode)
            rospy.wait_for_service(self._srv_name, timeout=timeout)
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Service {self._srv_name} setup failed: {err}")

    def initialise(self):
        self._srv_setup = False
        self._srv_called = False

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
        if self._res and self._res.success:
            rospy.loginfo(f"Success: {self._res.msg}")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
