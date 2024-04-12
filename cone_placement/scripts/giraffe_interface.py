#!/usr/bin/env python

from __future__ import annotations # for type hinting

import rospy
import actionlib
import numpy as np
import scipy as sc
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from move_action.msg import MoveAction, MoveGoal, MoveFeedback
from actionlib_msgs.msg import GoalStatus

from base_robot_interface import MoveBase, Move
import utils


# TODO make Move into action (?)
class GiraffeMove(Move):
    def __init__(
        self,
        goal_pose: Pose,
        timeout: rospy.Rate = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        super().__init__(goal_pose)
        print(f"Goal pose {goal_pose}")
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def init_move(self) -> bool:
        """
        check if within goal tolerance, then move based on distance
        first x, then y, then turn in yaw
        """
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        self._move_x()
        self._move_y()
        self._turn_yaw()

        # disp_xy, disp_th = self.displacement_from_pose(self.goal_pose)
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        rospy.logerr(
            f"cur ({curr_x:.2f},{curr_y:.2f}) [m,m] goal ({goal_x},{goal_y})[m,m]"
        )

        if self.at_goal_x() and self.at_goal_y() and self.at_goal_yaw():
            rospy.logwarn("Success")
            return True
        else:
            rospy.logwarn("Failure")
            return False

    def compute_twist_x(self) -> Twist:
        twist_msg = Twist()

        goal_x = self.goal_pose.position.x
        current_x = self.current_pose.position.x
        delta_x = goal_x - current_x

        twist_msg.linear.x = np.clip(delta_x, -self.max_lin_vel, self.max_lin_vel)
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        rospy.logwarn(f"sending velocity {twist_msg.linear.x:.2f}[m/s] to x")

        return twist_msg

    def compute_twist_y(self) -> Twist:
        twist_msg = Twist()

        goal_y = self.goal_pose.position.y
        current_y = self.current_pose.position.y
        delta_y = goal_y - current_y

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = np.clip(delta_y, -self.max_lin_vel, self.max_lin_vel)
        twist_msg.angular.z = 0.0
        rospy.logwarn(f"sending velocity {twist_msg.linear.y:.2f}[m/s] to y")

        return twist_msg

    def compute_twist_yaw(self) -> Twist:
        twist_msg = Twist()

        goal_quat = utils.array_from_pose(self.goal_pose)[3:]
        goal_yaw = utils.angle_from_quaternion(goal_quat)
        rospy.logerr(f"goal_yaw {goal_yaw}")

        current_quat = utils.array_from_pose(self.current_pose)[3:]
        current_yaw = utils.angle_from_quaternion(current_quat)

        rospy.logwarn(f"[current, goal] ({current_yaw:.2f}, {goal_yaw:.2f})[rad,rad]")
        delta_yaw = goal_yaw - current_yaw
        delta_yaw = utils.wrap_angle(delta_yaw)
        rospy.logwarn(f"distance from goal {delta_yaw:.2f}[rad]")

        gain = 1.0

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = np.clip(
            delta_yaw * gain, -self.max_ang_vel, self.max_ang_vel
        )
        rospy.logwarn(f"sending velocity {twist_msg.angular.z:.2f}[rad/s] to yaw")

        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_x(self) -> bool:
        goal_x = self.goal_pose.position.x
        current_x = self.current_pose.position.x

        rospy.logwarn(f"X[current, goal] ({current_x:.2f}, {goal_x:.2f}) [m,m]")

        return abs(goal_x - current_x) < self.tol_lin

    def at_goal_y(self) -> bool:
        goal_y = self.goal_pose.position.y
        current_y = self.current_pose.position.y

        rospy.logwarn(f"Y[current, goal] ({current_y:.2f}, {goal_y:.2f}) [m,m]")
        
        return abs(goal_y - current_y) < self.tol_lin

    def at_goal_yaw(self) -> bool:
        goal_quat = utils.array_from_pose(self.goal_pose)[3:]
        goal_yaw = utils.angle_from_quaternion(goal_quat)
        goal_yaw = utils.wrap_angle(goal_yaw)

        current_quat = utils.array_from_pose(self.current_pose)[3:]
        current_yaw = utils.angle_from_quaternion(current_quat)
        current_yaw = utils.wrap_angle(current_yaw)

        rospy.logwarn(f"Yaw[current, goal] ({current_yaw:.2f}, {goal_yaw:.2f}) [rad,rad]")

        return abs(goal_yaw - current_yaw) < self.tol_ang

    def displacement_from_pose(
        self, target_pose: np.array | Pose
    ) -> tuple[np.array, np.array]:
        """Get distance between target and current pose [m, deg]"""
        if type(target_pose) == Pose:
            target_pose = utils.array_from_pose(target_pose)

        target_pose_xy = target_pose[:2]
        target_pose_th = utils.angle_from_quaternion(target_pose[3:], "yaw")
        target_pose_th = utils.wrap_angle(target_pose_th)

        current_pose = self.current_pose
        if type(current_pose) == Pose:
            current_pose = utils.array_from_pose(current_pose)

        current_pose_xy = current_pose[:2]
        current_pose_th = utils.angle_from_quaternion(current_pose[3:], "yaw")
        current_pose_th = utils.wrap_angle(current_pose_th)

        disp_xy = round(np.linalg.norm(current_pose_xy - target_pose_xy) - 0.5, 3)
        delta_th = target_pose_th - current_pose_th

        disp_th = round(delta_th, 3)
        disp_th = utils.wrap_angle(disp_th)  # wrapping around [-180,180]

        rospy.logwarn(
            f"Robot at distance {abs(disp_xy):.2f}[m] and {abs(disp_th):.2f}[rad] from target"
        )

        return disp_xy, disp_th

    def _move_x(self) -> None:

        while not self.at_goal_x():
            twist_msg = self.compute_twist_x()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        self.twist_pub.publish(self.compute_empty_twist())

    def _move_y(self) -> None:

        while not self.at_goal_y():
            twist_msg = self.compute_twist_y()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        self.twist_pub.publish(self.compute_empty_twist())

    def _turn_yaw(self) -> None:

        while not self.at_goal_yaw():
            twist_msg = self.compute_twist_yaw()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        self.twist_pub.publish(self.compute_empty_twist())

def main():
    rospy.init_node("move_base_action")
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
            giraffe_move = GiraffeMove(goal_pose)
            rospy.sleep(1)
            giraffe_move.init_move()

            move_base_running = False


if __name__ == "__main__":
    main()
