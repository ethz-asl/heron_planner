#! /usr/bin/env python

import rospy
import actionlib
from heron_msgs.msg import GraspAction, GraspGoal

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

#  from giraffe_interface import GiraffeMoveBaseClient,GiraffeMove


class GraspClient:
    def __init__(self) -> None:
        node_name_ = rospy.get_param("~grasp_node", "/grasp_node")
        self._client = actionlib.SimpleActionClient(node_name_, GraspAction)

        rospy.loginfo(f"Connecting to {node_name_}...")
        self._client.wait_for_server(rospy.Duration(5.0))

    def init(self, goal_pose: Pose, ref_frame: str = "map") -> None:
        """
        Move the robot to a target pose.
        """
        goal = GraspGoal()
        print(f"frame_id: {goal.target_pose.header.frame_id}")
        print(f"time: {goal.target_pose.header.stamp}")
        print(f"pose: {goal.target_pose.pose}")

        goal.target_pose.header.frame_id = ref_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        # send the goal
        rospy.loginfo(f"Sending the move goal")
        self._client.send_goal(goal)

    def get_status(self) -> int:
        """
        get move_base status
        https://docs.ros.org/en/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
        """
        if self._client.get_state() == GoalStatus.SUCCEEDED:
            rospy.logwarn("Successfully reached the goal")
            return GoalStatus.SUCCEEDED
        else:
            return self._client.get_state()

    def cancel_goal(self) -> None:
        """
        Cancel the current goal
        """
        rospy.loginfo("Cancelling the move goal")
        self._client.cancel_goal()


def main():
    rospy.init_node("grasp_client")

    goal_pose = Pose()
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.8
    goal_pose.orientation.w = 1.0

    _client = GraspClient()
    _client.init(goal_pose)

    _running = True
    while _running:
        status = _client.get_status()
        if (
            status == GoalStatus.SUCCEEDED
            or status == GoalStatus.REJECTED
            or status == GoalStatus.ABORTED
        ):
            _running = False

    rospy.loginfo("Move finished")


if __name__ == "__main__":
    main()
