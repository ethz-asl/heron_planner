#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped

from moma_actions.giraffe_interface import GiraffeAtPose, GiraffeTopicClient, GiraffeParamClient, GiraffeComponentStatus
from moma_actions.move_action_client import MoveClient
from moma_actions.move_base_action_client import MoveBaseClient
from moma_actions.trigger_action_client import TriggerComponentClient
from moma_utils.ros.panda_client import PandaArmClient, PandaGripperClient
from grasp_node import GraspExecutionAction

import heron_utils.gazebo_utils as gazebo_utils
import heron_utils.transform_utils as utils

class MoveBase(pt.behaviour.Behaviour):
    """
    send goal to move_base action
    """

    def __init__(self, name: str, goal_pose: Pose, ref_frame: str = "map"):
        super().__init__(name)
        self._client = MoveBaseClient()
        self._goal_pose = goal_pose
        self._ref_frame = ref_frame

    def initialise(self):
        self._client.init_move_base(self._goal_pose, self._ref_frame)

    def update(self) -> pt.common.Status:
        status = self._client.get_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self._client.cancel_goal()


class Move(pt.behaviour.Behaviour):
    """
    Make small discrete adjustment movements to goal pose or goal distance

    1. combined (goal: Pose) -> linear x, linear y, and angular z (yaw)
    2. direction (distance: float) -> specific direction cmd
    directions: [forward, reverse, left, right, ccw, cw]
    """

    def __init__(self, name: str, goal_direction: str = 'combined', goal_pose: Pose = Pose(), goal_distance : float = 0.0, ref_frame: str = "map"):
        super().__init__(name)
        self._client = MoveClient()
        self._goal_direction = goal_direction
        self._goal_pose = goal_pose
        self._goal_distance = goal_distance
        self._ref_frame = ref_frame

    def initialise(self):
        if self._goal_direction == 'combined':
            self._client.send_combined_goal(self._goal_pose, self._ref_frame)
        else:
            self._client.send_distance_goal(self._goal_direction, self._goal_distance)

    def update(self) -> pt.common.Status:
        status = self._client.get_status()
        # rospy.logerr(f"move_status {status}")
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self._client.cancel_goal()

class TriggerComponent(pt.behaviour.Behaviour):
    """
    send a cmd to one of the robot components

    e.g. painting, roller, led
    """
    def __init__(self, name: str, component: str, cmd: str):
        super().__init__(name)
        self._client = TriggerComponentClient()
        self._component = component
        self._cmd = cmd

    def initialise(self):
        """
        here add the different kind of components or just send the goal
        """
        self._client.send_goal(self._component, self._cmd)

    def update(self):
        status = self._client.get_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE
 
    def terminate(self, new_status):
        if new_status == pt.common.Status.INVALID:
            self._client.cancel_goal()

class MoveArm(pt.behaviour.Behaviour):
    def __init__(self, name: str, goal_pose: Pose):
        super().__init__(name)
        self._client = PandaArmClient()
        self._goal_pose = goal_pose

    def initialise(self):
        self._client.goto(self._goal_pose)

    def update(self):
        # status = self._client.
        return super().update()

    def terminate(self, new_status):
        return super().terminate(new_status)


class GraspObject(pt.behaviour.Behaviour):
    def __init__(self, name: str, goal_pose: Pose):
        super().__init__(name)
        self._client = GraspExecutionAction()
        self._goal_pose = goal_pose

    def initialise(self):
        pass

    def update(self):
        pass

class RobotAtPose(pt.behaviour.Behaviour):
    def __init__(self, name: str, goal_pose: Pose, tol: float = 0.01):
        super().__init__(name)
        self._goal_pose = goal_pose
        self._tol = tol
        robot_name = rospy.get_param("~robot_name", "giraffe")
        if robot_name == "giraffe":   
            self._client = GiraffeAtPose()
        else:
            rospy.logerr("Not implemented robot besides giraffe")
            raise(NotImplementedError)
        
    def update(self):
        if self._client.at_pose(self._goal_pose, self._tol):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE
        
class HatchUp(pt.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        robot_name = rospy.get_param("~robot_name", "giraffe")
        if robot_name == "giraffe":   
            self._client = GiraffeComponentStatus("hatch")
        else:
            rospy.logerr("Not implemented robot besides giraffe")
            raise(NotImplementedError)

    def update(self):
        if self._client.is_hatch_up():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class RollerUp(pt.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        robot_name = rospy.get_param("~robot_name", "giraffe")
        if robot_name == "giraffe":   
            self._client = GiraffeComponentStatus("roller")
        else:
            rospy.logerr("Not implemented robot besides giraffe")
            raise(NotImplementedError)

    def update(self):
        if self._client.is_roller_up():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

# class SprayerOff(pt.behaviour.Behaviour):
#     def __init__(self, name: str) -> None:
#         super().__init__(name)
#         robot_name = rospy.get_param("~robot_name", "giraffe")
#         if robot_name == "giraffe":   
#             self._client = GiraffeComponentStatus("sprayer")
#         else:
#             rospy.logerr("Not implemented robot besides giraffe")
#             raise(NotImplementedError)

#     def update(self):
#         if self._client.is_sprayer_off():
#             return pt.common.Status.SUCCESS
#         else:
#             return pt.common.Status.FAILURE

class RSequence(pt.composites.Selector):
    """
    Rsequence for py_trees.
    Reactive sequence overidding sequence with memory, py_trees' only available sequence.
    Author: Chrisotpher Iliffe Sprague, sprague@kth.se
    """

    def __init__(self, name: str = "Sequence", children: List[Any] = None):
        super().__init__(name=name, children=children)

    def tick(self):
        """
        Run the tick behaviour for this selector.
        Note that the status of the tick is always determined by its children,
        not by the user customized update function.
        Yields
        ------
            class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children.
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != pt.common.Status.RUNNING:
            # selectors dont do anything specific on initialisation
            #   - the current child is managed by the update, never needs to be 'initialised'
            # run subclass (user) handles
            self.initialise()
        # run any work designated by a customized instance of this class
        self.update()
        previous = self.current_child
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child and (
                    node.status == pt.common.Status.RUNNING
                    or node.status == pt.common.Status.FAILURE
                ):
                    self.current_child = child
                    self.status = node.status
                    if previous is None or previous != self.current_child:
                        # we interrupted, invalidate everything at a lower priority
                        passed = False
                        for sibling in self.children:
                            if (
                                passed
                                and sibling.status != pt.common.Status.INVALID
                            ):
                                sibling.stop(pt.common.Status.INVALID)
                            if sibling == self.current_child:
                                passed = True
                    yield self
                    return
        # all children succeded,
        # set succeed ourselves and current child to the last bugger who failed us
        self.status = pt.common.Status.SUCCESS
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self
