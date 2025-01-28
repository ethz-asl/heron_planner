#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

import heron_utils.transform_utils as utils

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from robot_simple_command_manager_msgs.srv import SetCommandString
from heron_msgs.srv import (
    ChangeRobotMode,
    ChangeRobotModeRequest,
    ChangeRobotModeResponse,
    TransformPose,
    TransformPoseRequest,
    TransformPoseResponse,
    FindOffset,
    FindOffsetRequest,
    FindOffsetResponse,
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
    def __init__(self, name: str, pose: PoseStamped):
        super().__init__(name)
        self._srv_name = "/robot/command_manager/command"
        # TODO define this w/ Raquel
        self._pose = pose.pose
        rospy.logwarn(f"pose @ init {self._pose}")
        self._req = None

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
 
        pose_arr = utils.array_from_pose(self._pose)
        rospy.logwarn(f"pose arr {pose_arr}")
        yaw = utils.angle_from_quaternion(pose_arr[3:])
        rospy.loginfo(f"Going to [{pose_arr[0], pose_arr[1], yaw}]")
        self._req = (
            "GOTO_GPS "
            + str(pose_arr[0])
            + " "
            + str(pose_arr[1])
            + " "
            + yaw
        )

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


class AtPose(pt.behaviour.Behaviour):
    def __init__(self, name: str, goal_pose: PoseStamped) -> None:
        super().__init__(name)
        self.goal_pose = goal_pose.pose
        self.current_pose = None

        self.recieved_odom = False
        self.at_pose = None

        rospy.Subscriber("/robot/odom", Odometry, self.odom_cb)

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose
        self.recieved_odom = True

    def initialise(self):
        if self.recieved_odom is True:
            self.at_pose = utils.at_pose(self.current_pose, self.goal_pose)

    def update(self) -> pt.common.Status:

        if not self.at_pose:
            rospy.loginfo(f"Failed: not at pose")
            return pt.common.Status.FAILURE
        elif self.at_pose:
            rospy.loginfo("Success, at pose")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class PoseTransformer(pt.behaviour.Behaviour):
    def __init__(
            self, 
            name: str, 
            pose: PoseStamped, 
            target_frame: str,
            topic_name: str = "/transformed_pose",
    ) -> None:
        super().__init__(name)
        self._srv_name = "/transform_pose"
        self._req = TransformPoseRequest()
        self._req.pose_in = pose
        self._req.target_frame = target_frame
        self._res = TransformPoseResponse()
        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, TransformPose)
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
            self._pub.publish(self._res.pose_out)

        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:

        if not self._srv_setup or not self._srv_called:
            rospy.loginfo(f"Failed: {self._srv_setup} and {self._srv_called}")
            return pt.common.Status.FAILURE
        if self._res and self._res.success:
            rospy.loginfo("Success")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
    
    

class OffsetFinder(pt.behaviour.Behaviour):
    def __init__(
            self, name: str, 
            pose: PoseStamped, 
            defect_type: float,
            topic_name: str =  "/offset_pose"       
    ) -> None:
        super().__init__(name)
        self._srv_name = "/robot/find_offset"
        self._req = FindOffsetRequest()
        self._res = FindOffsetResponse()

        self._req.defect_pose = pose
        self._req.defect_type = defect_type

        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)

    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._srv_name, FindOffset)
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

            self._pub.publish(self._res.offset_pose)
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:

        if not self._srv_setup or not self._srv_called:
            rospy.loginfo(f"Failed: {self._srv_setup} and {self._srv_called}")
            return pt.common.Status.FAILURE
        if self._res and self._res.success:
            rospy.loginfo("Success")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
