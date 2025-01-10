#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

from robot_simple_command_manager_msgs.srv import SetCommandString

from std_msgs.msg import Empty


class Cancel(pt.behaviour.Behaviour):
    """need to send this before sending a new cmd, new cmds do not overwrite old"""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_manager/cancel"
        self._req = Empty()

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


class OpenDeposit(pt.behaviour.Behaviour):
    def __init__(self, name: str, deposit_num: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_sequencer/command"
        self._req = "DEPOSIT_" + deposit_num

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


class LiftRoller(pt.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_sequencer/command"
        self._req = "LIFT_ROLLER"

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


class LowerRoller(pt.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_sequencer/command"
        self._req = "LOWER_ROLLER"

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

    def update(self) -> pt.common.Status:

        if not self._srv_setup or not self._srv_called:
            rospy.loginfo(f"Failed: {self._srv_setup} and {self._srv_called}")
            return pt.common.Status.FAILURE
        if self._res and self._res.ret.success:
            rospy.loginfo("Success")
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class Blow(pt.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._srv_name = "/robot/command_sequencer/command"
        self._req = "BLOW"

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
