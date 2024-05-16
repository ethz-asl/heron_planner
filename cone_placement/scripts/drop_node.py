#!/usr/bin/env python3

from actionlib import SimpleActionServer
import numpy as np
import rospy

from grasp_demo.msg import DropAction, DropResult
from moma_utils.transform import Rotation, Transform
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda_client import PandaGripperClient


class DropActionNode(object):
    """Drops the object back into the workspace with a set offset."""

    def __init__(self):
        self.load_parameters()
        self.moveit = MoveItClient("panda_arm")
        self.gripper = PandaGripperClient()
        self.action_server = SimpleActionServer(
            "drop_action",
            DropAction,
            execute_cb=self.drop_object,
            auto_start=False,
        )
        self.action_server.start()

        rospy.loginfo("Drop action server ready")

    def load_parameters(self):
        self.velocity_scaling = rospy.get_param("heron_demo/arm_velocity_scaling_drop")

    def drop_object(self):
        rospy.loginfo("Dropping object")
        drop_joints = rospy.get_param("heron_demo/drop_joints")
        self.moveit.goto(drop_joints, velocity_scaling=self.velocity_scaling)
        self.gripper.release()
        self.action_server.set_succeeded(DropResult())


def main():
    rospy.init_node("drop_action_node")
    DropActionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
