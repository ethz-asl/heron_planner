#!/usr/bin/env python3

from actionlib import SimpleActionServer
import numpy as np
import rospy

from heron_msgs.msg import DropAction, DropResult, DropGoal
from moma_utils.transform import Rotation, Transform
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda_client import PandaArmClient, PandaGripperClient


class DropActionNode(object):
    """Drops the object back into the workspace with a set offset."""

    def __init__(self):
        self.load_parameters()
        self.moveit = PandaArmClient(self.arm_id)
        self.gripper = PandaGripperClient(self.gripper_id)
        self.action_server = SimpleActionServer(
            "drop_action",
            DropAction,
            execute_cb=self.drop_object,
            auto_start=False,
        )
        self.action_server.start()

        rospy.loginfo("Drop action server ready")

    def load_parameters(self):
        self.arm_id = rospy.get_param("heron_demo/arm_id", "panda_arm")
        self.gripper_id = rospy.get_param(
            "heron_demo/gripper_id", "panda/franka_gripper/"
        )
        self.velocity_scaling = rospy.get_param(
            "heron_demo/arm_velocity_scaling_drop", 0.3
        )

    def drop_object(self, drop_goal : DropGoal):
        rospy.loginfo("Dropping object")
        rospy.loginfo(f"Drop joints: {drop_goal.joints.data}")
        self.moveit.goto(list(drop_goal.joints.data), vel_scale=self.velocity_scaling)
        self.gripper.release()
        self.action_server.set_succeeded(DropResult())


def main():
    rospy.init_node("drop_node")
    DropActionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
