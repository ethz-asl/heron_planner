#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any

from geometry_msgs.msg import Pose, PoseStamped

import giraffe_interface
from move_action_client import MoveClient
from move_base_action_client import MoveBaseClient
from moma_utils.ros.panda_client import PandaArmClient, PandaGripperClient
from grasp_node import GraspExecutionAction

# TODO implement import heron_interface


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
    Make small discrete adjustment movements to goal pose
    1. linear x
    2. linear y
    3. angular z (yaw)
    """

    def __init__(self, name: str, goal_pose: Pose, ref_frame: str = "map"):
        super().__init__(name)
        self._client = MoveClient()
        self._goal_pose = goal_pose
        self._ref_frame = ref_frame

    def initialise(self):
        self._client.init_move(self._goal_pose, self._ref_frame)

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

class MovePredefinedPath(pt.behaviour.Behaviour):
    """
    From a start pose, start a predefined path trajectory for the base

    :param start_pose [Pose]: the starting postion the trajectory will start from
    :param path [str]: the type of path,  [pot_hole]    
    """
    def __init__(self, name: str, start_pose: Pose, path: str = "pot_hole" ,ref_frame: str = "map"):
        super().__init__(name)
        self._client = MovePredefinedPathClient()
        self._start_pose = start_pose
        self._path = path
        self._ref_frame = ref_frame

    def initialise(self):
        self._client.init_predefined_move(self._start_pose, self._path, self._ref_frame)

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
