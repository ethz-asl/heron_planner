#! /usr/bin/env python

import rospy
import actionlib
from heron_msgs.msg import MoveAction, MoveGoal, MoveFeedback

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

from giraffe_interface import GiraffeMoveBaseClient, GiraffeMove

class MoveClient():
    def __init__(self) -> None:
        #  self._action = heron_msgs.msg.MoveAction
        #  self._goal = heron_msgs.msg.MoveGoal
        #  self._feedback = heron_msgs.msg.MoveFeedback

        node_name_ = rospy.get_param("~move_node", "/move")
        self.move_client = actionlib.SimpleActionClient(node_name_, MoveAction)

        rospy.loginfo(f"Connecting to {node_name_}...")
        self.move_client.wait_for_server(rospy.Duration(5.0))

    def init_move(
        self,
        goal_pose: Pose,
        ref_frame: str = "map",
    ) -> None:
        """
        Move the robot to a target pose.
        """
        goal = MoveGoal()

        goal.target_pose.header.frame_id = ref_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        # send the goal
        rospy.loginfo("Sending the move goal")
        self.move_client.send_goal(goal)

    def get_move_status(self) -> int:
        """
        get move_base status
        https://docs.ros.org/en/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
        """
        if self.move_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.logwarn("Successfully reached the goal")
            return GoalStatus.SUCCEEDED
        else:
            return self.move_client.get_state()
            
        
def main():
    rospy.init_node("move_client")
    move_base_client = GiraffeMoveBaseClient()

    goal_pose = Pose()
    goal_pose.position.x = 2.0
    goal_pose.position.y = -1.0
    goal_pose.orientation.w = 1.0

    rospy.loginfo("Sending goal to robot")
    move_base_client.init_move_base(goal_pose)

    move_base_running = True

    while move_base_running:
        status = move_base_client.get_move_base_status()
        if (
            status == GoalStatus.SUCCEEDED
            or status == GoalStatus.REJECTED
            or status == GoalStatus.ABORTED
        ):

            move_base_running = False
    
    move_client = MoveClient()
    move_client.init_move(goal_pose)
    
    move_running = True
    while move_running:
        status = move_client.get_move_status()
        if (
            status == GoalStatus.SUCCEEDED
            or status == GoalStatus.REJECTED
            or status == GoalStatus.ABORTED
        ):

            move_running = False
            rospy.loginfo("Move finished")

if __name__ == '__main__':
    main()
