#!/usr/bin/env python

import rospy
import actionlib
import py_trees as pt

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from robot_simple_command_manager_msgs.srv import SetCommandString
from heron_msgs.msg import (
    MoveToAction, MoveToGoal,
    MoveToPoseAction, MoveToPoseGoal,
    PickupFromAction, PickupFromGoal,
    PlaceOnAction, PlaceOnGoal,
)

class MoveTo(pt.behaviour.Behaviour):
    def __init__(self, name: str, pose_name: str) -> None:
        super().__init__(name)
        self._node_name = "/robot/arm/move_to"
        self._goal = MoveToGoal(to=pose_name)
        self._client = None
        self._result = None
        self._feedback_recieved = None
        
    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = actionlib.SimpleActionClient(self._node_name, MoveToAction)
            if not self._client.wait_for_server(timeout=timeout):
                rospy.logerr(f"Action {self._node_name} setup failed: {err}")
                return False
            rospy.loginfo(f"Action {self._node_name} is ready")
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Action {self._node_name} setup failed: {err}")
            return False
        
    def initialise(self):
        
        if not self.setup():
            rospy.logwarn(f"Action {self._node_name} is not initialised")
            return

        rospy.loginfo(f"Sending goal to {self._node_name} :{self._goal}")
        self._client.send_goal(self._goal, feedback_cb=self.feedback_cb)
        self._feedback_recieved = False
        self._result = None

    def update(self) -> pt.common.Status:
        if not self._client:
            rospy.logwarn("Action client is not initialized.")
            return pt.common.Status.FAILURE

        if self._client.get_state() == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Action is in progress...")
            return pt.common.Status.RUNNING
        elif self._client.get_state() in [actionlib.GoalStatus.SUCCEEDED]:
            rospy.loginfo("Action succeeded!")
            self._result = self._client.get_result()
            return pt.common.Status.SUCCESS
        elif self._client.get_state() in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            rospy.logerr("Action failed or was rejected.")
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("Action is in an unknown state.")
            return pt.common.Status.RUNNING
        
    def terminate(self, new_status: pt.common.Status):
        if self._client and new_status == pt.common.Status.FAILURE:
            rospy.loginfo("Cancelling action goal")
            self._client.cancel_goal()
        
    def feedback_cb(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")
        self._feedback_recieved = True
        

class MoveToPose(pt.behaviour.Behaviour):
    def __init__(self, name: str, pose: PoseStamped) -> None:
        super().__init__(name)
        self._node_name = "/robot/arm/move_to_pose"
        self._goal = MoveToPoseGoal(pose=pose)
        self._client = None
        self._result = None
        self._feedback_recieved = None
        
    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = actionlib.SimpleActionClient(self._node_name, MoveToPoseAction)
            if not self._client.wait_for_server(timeout=timeout):
                rospy.logerr(f"Action {self._node_name} setup failed: {err}")
                return False
            rospy.loginfo(f"Action {self._node_name} is ready")
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Action {self._node_name} setup failed: {err}")
            return False
        
    def initialise(self):
        
        if not self.setup():
            rospy.logwarn(f"Action {self._node_name} is not initialised")
            return

        rospy.loginfo(f"Sending goal to {self._node_name} :{self._goal}")
        self._client.send_goal(self._goal, feedback_cb=self.feedback_cb)
        self._feedback_recieved = False
        self._result = None

    def update(self) -> pt.common.Status:
        if not self._client:
            rospy.logwarn("Action client is not initialized.")
            return pt.common.Status.FAILURE

        if self._client.get_state() == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Action is in progress...")
            return pt.common.Status.RUNNING
        elif self._client.get_state() in [actionlib.GoalStatus.SUCCEEDED]:
            rospy.loginfo("Action succeeded!")
            self._result = self._client.get_result()
            return pt.common.Status.SUCCESS
        elif self._client.get_state() in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            rospy.logerr("Action failed or was rejected.")
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("Action is in an unknown state.")
            return pt.common.Status.RUNNING
        
    def terminate(self, new_status: pt.common.Status):
        if self._client and new_status == pt.common.Status.FAILURE:
            rospy.loginfo("Cancelling action goal")
            self._client.cancel_goal()
        
    def feedback_cb(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")
        self._feedback_recieved = True

class PickUpFrom(pt.behaviour.Behaviour):
    def __init__(self, name: str, loc: str) -> None:
        super().__init__(name)
        self._node_name = "/robot/arm/pickup_from"
        self._goal = PickUpFrom(location=loc)
        self._client = None
        self._result = None
        self._feedback_recieved = None
        
    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = actionlib.SimpleActionClient(self._node_name, PickupFromAction)
            if not self._client.wait_for_server(timeout=timeout):
                rospy.logerr(f"Action {self._node_name} setup failed: {err}")
                return False
            rospy.loginfo(f"Action {self._node_name} is ready")
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Action {self._node_name} setup failed: {err}")
            return False
        
    def initialise(self):
        
        if not self.setup():
            rospy.logwarn(f"Action {self._node_name} is not initialised")
            return

        rospy.loginfo(f"Sending goal to {self._node_name} :{self._goal}")
        self._client.send_goal(self._goal, feedback_cb=self.feedback_cb)
        self._feedback_recieved = False
        self._result = None

    def update(self) -> pt.common.Status:
        if not self._client:
            rospy.logwarn("Action client is not initialized.")
            return pt.common.Status.FAILURE

        if self._client.get_state() == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Action is in progress...")
            return pt.common.Status.RUNNING
        elif self._client.get_state() in [actionlib.GoalStatus.SUCCEEDED]:
            rospy.loginfo("Action succeeded!")
            self._result = self._client.get_result()
            return pt.common.Status.SUCCESS
        elif self._client.get_state() in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            rospy.logerr("Action failed or was rejected.")
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("Action is in an unknown state.")
            return pt.common.Status.RUNNING
        
    def terminate(self, new_status: pt.common.Status):
        if self._client and new_status == pt.common.Status.FAILURE:
            rospy.loginfo("Cancelling action goal")
            self._client.cancel_goal()
        
    def feedback_cb(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")
        self._feedback_recieved = True

class PlaceOn(pt.behaviour.Behaviour):
    def __init__(self, name: str, loc : str) -> None:
        super().__init__(name)
        self._node_name = "/robot/arm/place_on"
        self._goal = PlaceOnGoal(location=loc)
        self._client = None
        self._result = None
        self._feedback_recieved = None
        
    def setup(self, timeout: float = 2.0) -> bool:
        try:
            self._client = actionlib.SimpleActionClient(self._node_name, PlaceOnAction)
            if not self._client.wait_for_server(timeout=timeout):
                rospy.logerr(f"Action {self._node_name} setup failed: {err}")
                return False
            rospy.loginfo(f"Action {self._node_name} is ready")
            return True
        except rospy.ROSException as err:
            rospy.logerr(f"Action {self._node_name} setup failed: {err}")
            return False
        
    def initialise(self):
        
        if not self.setup():
            rospy.logwarn(f"Action {self._node_name} is not initialised")
            return

        rospy.loginfo(f"Sending goal to {self._node_name} :{self._goal}")
        self._client.send_goal(self._goal, feedback_cb=self.feedback_cb)
        self._feedback_recieved = False
        self._result = None

    def update(self) -> pt.common.Status:
        if not self._client:
            rospy.logwarn("Action client is not initialized.")
            return pt.common.Status.FAILURE

        if self._client.get_state() == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Action is in progress...")
            return pt.common.Status.RUNNING
        elif self._client.get_state() in [actionlib.GoalStatus.SUCCEEDED]:
            rospy.loginfo("Action succeeded!")
            self._result = self._client.get_result()
            return pt.common.Status.SUCCESS
        elif self._client.get_state() in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            rospy.logerr("Action failed or was rejected.")
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("Action is in an unknown state.")
            return pt.common.Status.RUNNING
        
    def terminate(self, new_status: pt.common.Status):
        if self._client and new_status == pt.common.Status.FAILURE:
            rospy.loginfo("Cancelling action goal")
            self._client.cancel_goal()
        
    def feedback_cb(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")
        self._feedback_recieved = True
