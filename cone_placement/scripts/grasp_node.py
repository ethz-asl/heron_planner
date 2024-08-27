#!/usr/bin/env python3

from actionlib import SimpleActionServer
from geometry_msgs.msg import Pose, PoseStamped
import rospy

from heron_msgs.msg import GraspAction
from moma_utils.ros.conversions import from_pose_msg, to_pose_stamped_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda_client import PandaArmClient, PandaGripperClient
from moma_utils.transform import Transform


class GraspExecutionAction(object):
    """Execute a grasp specified by the action goal using MoveIt."""

    def __init__(self):
        self.load_parameters()
        # self.moveit = MoveItClient(self.arm_id)
        self.arm = PandaArmClient(self.arm_id)
        self.gripper = PandaGripperClient(self.gripper_id)

        self.action_server = SimpleActionServer(
            "grasp_execution_action",
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        self.moveit_target_pub = rospy.Publisher(
            "target", PoseStamped, queue_size=10
        )

        rospy.loginfo("Grasp action server ready")

    def load_parameters(self):
        self.arm_id = rospy.get_param("heron_demo/arm_id", "panda_arm")
        self.gripper_id = rospy.get_param(
            "heron_demo/gripper_id", "panda/franka_gripper/"
        )
        self.base_frame = rospy.get_param(
            "heron_demo/base_frame_id", "base_footprint"
        )
        self.velocity_scaling = rospy.get_param(
            "heron_demo/arm_velocity_scaling_grasp", 0.2
        )
        self.ee_grasp_offset_z = rospy.get_param(
            "heron_demo/ee_grasp_offset_z", -0.04
        )

    def execute_cb(self, goal: PoseStamped):
        rospy.loginfo("Received grasp pose")
        T_base_grasp = from_pose_msg(goal.target_grasp_pose.pose)

        T_grasp_ee_offset = Transform.translation(
            [0.0, 0.0, -self.ee_grasp_offset_z]
        )

        rospy.loginfo("Executing grasp")
        self.gripper.release()
        self.arm.goto("home", self.velocity_scaling)

        rospy.loginfo("Moving to pregrasp pose")
        target = (
            T_base_grasp
            * Transform.translation([0, 0, -0.04])
            * T_grasp_ee_offset
        )
        self.moveit_target_pub.publish(
            to_pose_stamped_msg(target, self.base_frame)
        )
        success = self.arm.goto(target, self.velocity_scaling)
        rospy.loginfo(f"success of pregrasp {success}")

        rospy.loginfo("Moving to grasp pose")
        target = T_base_grasp * T_grasp_ee_offset
        self.moveit_target_pub.publish(
            to_pose_stamped_msg(target, self.base_frame)
        )
        self.arm.go_to_pose_goal_cartesian(target, self.velocity_scaling)

        if self.arm.has_error:
            rospy.loginfo("Robot error. Aborting.")
            self.action_server.set_aborted()
            return

        rospy.loginfo("Attempting grasp")
        self.gripper.grasp()

        if self.arm.has_error:
            rospy.loginfo("Robot error. Aborting.")
            self.action_server.set_aborted()
            return

        rospy.loginfo("Lifting object")
        target = (
            Transform.translation([0, 0, 0.2])
            * T_base_grasp
            * T_grasp_ee_offset
        )
        self.moveit_target_pub.publish(
            to_pose_stamped_msg(target, self.base_frame)
        )
        self.arm.go_to_pose_goal_cartesian(target, self.velocity_scaling)

        if self.gripper.read() > 0.002:
            rospy.loginfo("Object grasped successfully")
            self.action_server.set_succeeded()
        else:
            rospy.logwarn("Nothing detected in gripper")
            self.action_server.set_aborted()

        #self.gripper.release()


def main():
    rospy.init_node("grasp_node")
    GraspExecutionAction()
    rospy.spin()


if __name__ == "__main__":
    main()
