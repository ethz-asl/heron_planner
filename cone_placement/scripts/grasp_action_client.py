#! /usr/bin/env python

import rospy
import actionlib
import copy
from heron_msgs.msg import GraspAction, GraspGoal

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

#  from giraffe_interface import GiraffeMoveBaseClient,GiraffeMove


class GraspClient:
    def __init__(self) -> None:
        action_name_ = rospy.get_param(
            "~grasp_action", "grasp_execution_action"
        )
        self._client = actionlib.SimpleActionClient(action_name_, GraspAction)

        rospy.loginfo(f"Connecting to {action_name_}...")
        self._client.wait_for_server(rospy.Duration(5.0))

    def init(self, goal_pose: Pose, ref_frame: str = "base_footprint") -> None:
        """
        Move the robot to a target pose.
        """
        goal = GraspGoal()
        goal.target_grasp_pose.header.frame_id = ref_frame
        goal.target_grasp_pose.header.stamp = rospy.Time.now()
        goal.target_grasp_pose.pose = goal_pose

        print(f"frame_id: {goal.target_grasp_pose.header.frame_id}")
        print(f"time: {goal.target_grasp_pose.header.stamp}")
        print(f"pose: {goal.target_grasp_pose.pose}")

        # send the goal
        rospy.loginfo(f"Sending the grasp goal")
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
    goal_pose.position.x = -0.15
    goal_pose.position.y = 0.1
    goal_pose.position.z = 0.30
    goal_pose.orientation.w = 1.0

    target_pose1 = Pose()
    target_pose1.position.x = 0.1
    target_pose1.position.y = -0.2
    target_pose1.position.z = 0.5
    target_pose1.orientation.x = -0.476214
    target_pose1.orientation.y = 0.485218
    target_pose1.orientation.z = -0.517232
    target_pose1.orientation.w = 0.519859

    goal = copy.deepcopy(target_pose1)
    _client = GraspClient()
    _client.init(goal)

    _running = True
    while _running and not rospy.is_shutdown():
        status = _client.get_status()
        if (
            status == GoalStatus.SUCCEEDED
            or status == GoalStatus.REJECTED
            or status == GoalStatus.ABORTED
        ):
            _running = False

    rospy.loginfo("Grasp finished finished")


if __name__ == "__main__":
    main()
