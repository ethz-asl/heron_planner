#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function, annotations
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import numpy as np
import scipy as sc
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation
import moma_utils.ros.conversion as moma_conv
from moma_utils.spartial import Transform

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(
    goal: list | PoseStamped | Pose,
    actual: list | PoseStamped | Pose,
    tolerance: float = 0.01,
) -> bool:
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveArmClient(object):
    """MoveIt Arm client for manipulator"""

    def __init__(self, group_name: str = "panda_arm") -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_client", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        named_targets = move_group.get_named_targets()
        print("============ Named targets %s" % named_targets)

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.named_targets = named_targets
        self.status: int = 0

    def goto(
        self,
        target: Transform | list | np.ndarray | str,
        vel_scale: float = 0.2,
        acc_scale: float = 0.2,
    ) -> bool:
        plan = self.plan(target, vel_scale, acc_scale)
        return self.execute(plan)

    def plan(
        self,
        target: Transform | list | np.ndarray | str,
        vel_scale: float = 0.2,
        acc_scale: float = 0.2,
    ) -> RobotTrajectory:
        self.move_group.set_max_velocity_scaling_factor(vel_scale)
        self.move_group.set_max_acceleration_scaling_factor(acc_scale)

        if isinstance(target, Transform):
            self.move_group.set_pose_target(moma_conv.to_pose_msg(target))
        elif isinstance(target, (list, np.ndarry)):
            self.move_group.set_joint_value_target(target)
        elif isinstance(target, str):
            self.move_group.set_named_target(target)
        else:
            rospy.logerr(f"Type of target goal, {type(target)}, is not valid")
            raise ValueError

        plan = self.move_group.plan()
        if type(plan) is tuple:
            plan = plan[1]

        return plan

    def execute(self, plan: RobotTrajectory) -> bool:
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def go_to_named_target(self, target: str) -> bool:
        """go to named target"""
        return self.go_to_joint_goal(self.move_group.get_named_target_values(target))

    def go_to_floor(self) -> bool:
        return self.go_to_named_target("floor")

    def go_to_home(self) -> bool:
        return self.go_to_named_target("home")

    def go_to_safe(self) -> bool:
        return self.go_to_named_target("safe")

    def go_to_joint_goal(self, joint_goal: dict) -> bool:
        """
        Planning to a Joint Goal
        """
        rospy.logerr(f"type of joint_goal: {type(joint_goal)}")
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints)

    def get_pose(self, position: np.array, orientation: np.array) -> Pose:
        pose_goal = Pose()
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]

        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        return pose_goal

    def go_to_pose_goal(self, pose_goal: Pose) -> bool:
        """
        Planning to a Pose Goal
        Plan a motion for this group to a desired pose for the
        end-effector:
        """
        self.move_group.set_pose_target(pose_goal)

        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose)

    def plan_cartesian_path(self, scale: float = 1.0) -> tuple[RobotTrajectory, float]:
        """"""
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        return plan, fraction

    def display_trajectory(self, plan: RobotTrajectory) -> None:
        """"""
        self.display_trajectory = DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(self.display_trajectory)

    def execute_plan(self, plan: RobotTrajectory) -> None:
        """
        Executing a Plan
        """
        self.move_group.execute(plan, wait=True)

    def cancel_goal(self) -> None:
        """Cancel the current goal"""
        rospy.loginfo("Cancelling MoveIt goal")
        self.move_group.stop()


def main():
    move = MoveArmClient()
    move.go_to_home()

    while True:
        try:
            print("")
            print("----------------------------------------------------------")
            print("Sampling sphere coordinates for wrist calibration")
            print("----------------------------------------------------------")
            print("Press 'h' to go to a home state")
            print("----------------------------------------------------------")
            print("Press 's' to go to a safe state")
            print("----------------------------------------------------------")
            print("Press 'f' to go to a floor state")
            print("----------------------------------------------------------")
            print("Press 'q' to stop sampling exit")

            print("")

            user_input = input("============ Choose an option: ")
            print("")

            if user_input == "h":
                move.go_to_home()
            elif user_input == "s":
                move.go_to_safe()
            elif user_input == "f":
                move.go_to_floor()
            elif user_input == "q":
                print("============ Sampling complete!")
                print("")
                break

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    main()
