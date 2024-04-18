#! /usr/bin/env python

import rospy
import actionlib
from heron_msgs.msg import MoveAction, MoveGoal, MoveFeedback

from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus

from giraffe_interface import GiraffeMove


class MoveServer(object):
    """
    Move action server, to move robot to specific position with tolerance
    """

    def __init__(self, name) -> None:
        self._action_name = name
        self._robot_name = rospy.get_param("~robot_name", "giraffe")

        self._action = MoveAction()
        self._goal = MoveGoal()
        self._feedback = MoveFeedback()

        self._server = actionlib.SimpleActionServer(
            self._action_name, MoveAction, self.execute_cb, auto_start=False
        )
        rospy.loginfo("Starting move action server")
        self._server.start()
        self._cancel_requested = False

    def execute_cb(self, goal: PoseStamped) -> None:
        rospy.loginfo(f"Recieved goal {goal.target_pose.pose}")

        goal_pose = Pose()
        goal_pose = goal.target_pose.pose
        move = None

        self._server.preempt_request = False
        feedback_pub = lambda pose: self._publish_feedback(pose)

        if self._robot_name == "giraffe":
            move = GiraffeMove(goal_pose, feedback_pub)
            rospy.sleep(2.0)
        elif self._robot_name == "heron":
            raise NotImplementedError

        while True:
            if (
                rospy.is_shutdown()
                or self._server.is_preempt_requested()
                or self._cancel_requested
            ):
                rospy.loginfo("move goal preempted")
                self._server.set_preempted()
                self._cancel_requested = True
                success = False
                break

            elif move.init_move():
                self._feedback.base_position.pose = move.current_pose
                self._server.publish_feedback(self._feedback)
                self._server.set_succeeded()
                success = True
                break
    def _publish_feedback(self, pose: Pose) -> None:
        """
        publish the current pose of the robot
        """
        self._feedback.base_position.pose = pose
        self._server.publish_feedback(self._feedback)
    def cancel_goal(self):
        """
        Cancel the move goal.
        """
        rospy.loginfo("Cancelling move goal")
        self._server.set_preempted()  # Preempt the server

        # If the server is not done, preempt the goal
        rospy.logerr(f"state {self._server.get_state()}")
        if self_server.get_state() == actionlib.GoalStatus.DONE:
            rospy.loginfo("Goal was already reached")
            return
        else:
            self._server.set_preempted()


def main():
    rospy.init_node("move")
    MoveServer(rospy.get_name())
    rospy.spin()


if __name__ == "__main__":
    main()
