#!/usr/bin/env python

import rospy
import actionlib
import py_trees
import py_trees_ros

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

#
#  class MoveBaseBehavior(py_trees.behaviour.Behaviour):
    #  def __init__(self, name="MoveBase"):
        #  super(MoveBaseBehavior, self).__init__(name)
#
        #  self.move_base_client = None
        #  self,goal_sent = False
#
    #  def setup(self):
        #  self.move_base_publisher = rospy.Publisher(
            #  "/move_base_simple/goal", PoseStamped, queue_size=1
        #  )
        #  self.move_base_client = py_trees_ros.action_clients.ActionClient(
            #  "move_base", MoveBaseAction
        #  )
#
#
    #  def update(self):
        #  if self.move_base_client.status == GoalStatus.PENDING:
            #  # Goal is pending, send a new goal
            #  goal = MoveBaseGoal()
            #  # Define your goal here, e.g., setting the goal pose
            #  goal.target_pose.header.frame_id = "map"
            #  goal.target_pose.pose.position.x = 1.0
            #  goal.target_pose.pose.position.y = 1.0
            #  goal.target_pose.pose.orientation.w = 1.0
            #  self.move_base_client.send_goal(goal)
            #  return py_trees.common.Status.RUNNING
        #  elif self.move_base_client.status == GoalStatus.ACTIVE:
            #  # Goal is active, wait for completion
            #  return py_trees.common.Status.RUNNING
        #  elif self.move_base_client.status == GoalStatus.SUCCEEDED:
            #  # Goal succeeded
            #  return py_trees.common.Status.SUCCESS
        #  else:
            #  # Goal failed or canceled
            #  return py_trees.common.Status.FAILURE
#
#
#  def create_behavior_tree():
    #  root = py_trees.composites.Sequence("MoveBaseSequence")
    #  move_base_behavior = MoveBaseBehavior()
    #  root.add_child(move_base_behavior)
    #  return root


def send_move_base_goal():

    # Create a move base action client
    
    try:
        move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        move_base_client.wait_for_server()
    except rospy.ServiceException as srv_exp:
        rospy.logwarn("No move_base service avaliable")
        rospy.logerr(srv_exp)

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.position.y = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    # Send the goal
    move_base_client.send_goal(goal)

    # Wait for the result
    move_base_client.wait_for_result()

    # Check the result
    if move_base_client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("MoveBase goal reached successfully!")
    else:
        rospy.logerr("MoveBase goal failed!")


def main():
    rospy.init_node("move_base_bt")
    send_move_base_goal()

if __name__ == "__main__":
    main()
