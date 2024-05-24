#! /usr/bin/env python

import rospy
import actionlib
from heron_msgs.msg import DropAction, DropGoal

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus

#  from giraffe_interface import GiraffeMoveBaseClient,GiraffeMove


class DropClient:
    def __init__(self) -> None:
        action_name_ = rospy.get_param("~drop_action", "drop_action")
        self._client = actionlib.SimpleActionClient(action_name_, DropAction)

        rospy.loginfo(f"Connecting to {action_name_}...")
        self._client.wait_for_server(rospy.Duration(5.0))

    def init(self, drop_joints : list = None) -> None:
        """
        Move the robot to a target pose.
        """
        if drop_joints == None:
            drop_joints = rospy.get_param("heron_demo/drop_joints")

        goal = DropGoal()
        goal.joints.data = drop_joints
        rospy.loginfo(f"Sending the drop goal")
        self._client.send_goal(goal)

    def get_status(self) -> int:
        """
        get status
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
    rospy.init_node("drop_client")

    _client = DropClient()
    _client.init()

    _running = True
    while _running and not rospy.is_shutdown():
        status = _client.get_status()
        if (
            status == GoalStatus.SUCCEEDED
            or status == GoalStatus.REJECTED
            or status == GoalStatus.ABORTED
        ):
            _running = False

    rospy.loginfo("Grasp finished dropping")


if __name__ == "__main__":
    main()
