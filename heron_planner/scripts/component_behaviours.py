#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

from robot_simple_command_manager_msgs.srv import SetCommandString

class ROSWait(pt.behaviour.Behaviour):
    def __init__(self, name: str, wait_time: float) -> None:
        super().__init__(name)
        self._wait_time = wait_time
        
    def initialise(self):
        self._rate = rospy.Rate()

    def update(self) -> pt.common.Status:
        self._rate.sleep(self._wait_time)
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
                return pt.common.Status.FAILURE
            if self._res and hasattr(self._res, "success") and self._res.success:
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
        self._srv_called = False
        self._res = None

        self.setup()

        try:
            rospy.loginfo(f"Sending command {self._req}")
            self._res = self._client(self._req)
            self._srv_called = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:
            if not self._srv_called:
                return pt.common.Status.FAILURE
            if self._res and hasattr(self._res, "success") and self._res.success:
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
        self._srv_called = False
        self._res = None

        self.setup()

        try:
            rospy.loginfo(f"Sending command {self._req}")
            self._res = self._client(self._req)
            self._srv_called = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:
        if not self._srv_called:
            return pt.common.Status.FAILURE
        if self._res and hasattr(self._res, "success") and self._res.success:
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
        self._srv_called = False
        self._res = None

        self.setup()

        try:
            rospy.loginfo(f"Sending command {self._req}")
            self._res = self._client(self._req)
            self._srv_called = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:
        if not self._srv_called:
            return pt.common.Status.FAILURE
        if self._res and hasattr(self._res, "success") and self._res.success:
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
            return pt.common.Status.FAILURE
        if self._res and hasattr(self._res, "success") and self._res.success:
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

        try:
            rospy.loginfo(f"Sending command {self._req}")
            self._res = self._client(self._req)
            self._srv_called = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:
        if not self._srv_setup or not self._srv_called:
            return pt.common.Status.FAILURE
        if self._res and hasattr(self._res, "success") and self._res.success:
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
        self._srv_called = False
        self._res = None

        self.setup()

        try:
            rospy.loginfo(f"Sending command {self._req}")
            self._res = self._client(self._req)
            self._srv_called = True
        except rospy.ServiceException as err:
            rospy.logerr(f"Service {self._srv_name} failed : {err}")
            self._srv_called = False

    def update(self) -> pt.common.Status:
        if not self._srv_called:
            return pt.common.Status.FAILURE
        if self._res and hasattr(self._res, "success") and self._res.success:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
 

class ServiceCall(pt.behaviour.Behaviour):
    """
    call ros service and handle success or failure.
    """

    def __init__(self, name: str, service_name: str, service_type: Any, request: Any):
        super().__init__(name)
        self._service_name = service_name
        self._service_type = service_type
        self._request = request
        self._client = None
        self._service_called = False
        self._service_response = None

    def setup(self, timeout: float) -> bool:
        try:
            self._client = rospy.ServiceProxy(self._service_name, self._service_type)
            rospy.wait_for_service(self._service_name, timeout=timeout)
            return True
        except rospy.ROSException as e:
            rospy.logerr(f"Service {self._service_name} setup failed: {e}")
            return False

    def initialise(self):
        self._service_called = False
        self._service_response = None
        try:
            rospy.loginfo(f"Calling service: {self._service_name}")
            self._service_response = self._client(self._request)
            self._service_called = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Service {self._service_name} call failed: {e}")
            self._service_called = False

    def update(self) -> pt.common.Status:
        if not self._service_called:
            return pt.common.Status.FAILURE
        if self._service_response and hasattr(self._service_response, "success") and self._service_response.success:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
