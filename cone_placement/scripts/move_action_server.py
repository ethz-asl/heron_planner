#! /usr/bin/env python

import rospy
import actionlib
from heron_msgs.msg import MoveAction, MoveGoal, MoveFeedback

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

from giraffe_interface import GiraffeMoveBaseClient, GiraffeMove

class MoveServer(object):
    """
    Move action server, to move robot to specific position with tolerance
    """
    def __init__(
        self,
        name,
    ) -> None:
        self._action_name = name
        self._robot_name = rospy.get_param("~robot_name", "giraffe")
        
        self._action = MoveAction()
        self._goal = MoveGoal()
        self._feedback = MoveFeedback()

        self._server = actionlib.SimpleActionServer(self._action_name, MoveAction, self.execute_cb, auto_start=False)
        rospy.loginfo("Starting move action server")
        self._server.start()

    def execute_cb(self, goal) -> None:

        rospy.loginfo(f"Recieved goal {goal.target_pose.pose}")

        goal_pose = goal.target_pose.pose
        move = None

        if self._robot_name == "giraffe":
            move = GiraffeMove(goal_pose)
            rospy.sleep(2.0)
        elif self._robot_name == "heron":
            raise NotImplementedError

        if self._server.is_preempt_requested():
            self._server.set_preempted() 
        elif move.init_move():
            self._server.set_succeeded()
        else:
            self._server.set_aborted()

        self._feedback.base_position = move.current_pose
        self._server.publish_feedback(self._feedback)
        
def main():

    rospy.init_node('move')
    MoveServer(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()