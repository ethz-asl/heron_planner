#!/usr/bin/env python

from __future__ import annotations  # for type hinting
from typing import Callable, Optional

import rospy
import actionlib
import numpy as np
import scipy as sc
from scipy.spatial.transform import Rotation

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# from move_action.msg import MoveAction, MoveGoal, MoveFeedback
from actionlib_msgs.msg import GoalStatus

from base_robot_interface import Move, PredefinedPath, PotholeFilling
from move_base_action_client import MoveBaseClient

# import moma_utils.transform_utils as utils # should migrate to moma_utils
import utils.transform_utils as utils


class GiraffeMove(Move):

    # TODO add local vs global to checking at_goal functions!!!
    def __init__(
        self,
        goal_pose: Pose,
        feedback_pub: Callable[[Pose], None],
        timeout: rospy.Rate = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        super().__init__(goal_pose)
        self.timeout = rospy.Rate(timeout)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.feedback_pub = feedback_pub
        rospy.sleep(1)  # wait for subscriber to latch

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def init_move(self) -> bool:
        """
        check if within goal tolerance, then move based on distance
        first x, then y, then turn in yaw
        """
        while not rospy.is_shutdown():
            self.feedback_pub(self.current_pose)

            if self.move():
                self.feedback_pub(self.current_pose)
                rospy.loginfo("Success, robot reached move goal")
                return True
            else:
                self.feedback_pub(self.current_pose)
                rospy.logerr("Failure, robot did not reach move goal")
                return False

        return False

    def compute_twist(self) -> Twist:
        twist_msg = Twist()

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        delta_x_global = goal_x - current_x
        delta_y_global = goal_y - current_y

        goal_quat = utils.array_from_pose(self.goal_pose)[3:]
        goal_yaw = utils.angle_from_quaternion(goal_quat)
        current_quat = utils.array_from_pose(self.current_pose)[3:]
        current_yaw = utils.angle_from_quaternion(current_quat)

        delta_x_local = (
            np.cos(current_yaw) * delta_x_global
            + np.sin(current_yaw) * delta_y_global
        )
        delta_y_local = (
            -np.sin(current_yaw) * delta_x_global
            + np.cos(current_yaw) * delta_y_global
        )

        delta_yaw = utils.wrap_angle(goal_yaw - current_yaw)

        twist_msg.linear.x = np.clip(
            delta_x_local, -self.max_lin_vel, self.max_lin_vel
        )
        twist_msg.linear.y = np.clip(
            delta_y_local, -self.max_lin_vel, self.max_lin_vel
        )
        twist_msg.angular.z = np.clip(
            delta_yaw, -self.max_ang_vel, self.max_ang_vel
        )

        rospy.loginfo(f"MoveX: Sending velocity {twist_msg.linear.x:.2f} [m/s] to x")
        rospy.loginfo(f"MoveY: Sending velocity {twist_msg.linear.y:.2f} [m/s] to y")
        rospy.loginfo(f"MoveYaw: Sending velocity {twist_msg.angular.z:.2f} [rad/s] to yaw")

        return twist_msg

    def move(self) -> bool:
        """
        move the robot towards the goal using local frame
        """
        if self.at_goal_x() and self.at_goal_y() and self.at_goal_yaw():
            return True

        while (
            not self.at_goal_x()
            or not self.at_goal_y()
            or not self.at_goal_yaw()
        ):
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist()
            self.twist_pub.publish(twist_msg)
            self.timeout.sleep()

        self.twist_pub.publish(self.compute_empty_twist())  # stop if at goal

        return self.at_goal_x() and self.at_goal_y() and self.at_goal_yaw()

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_x(self) -> bool:
        goal_x = self.goal_pose.position.x
        current_x = self.current_pose.position.x

        # rospy.logwarn(f"X[current, goal] ({current_x:.2f}, {goal_x:.2f}) [m,m]")

        return abs(goal_x - current_x) < self.tol_lin

    def at_goal_y(self) -> bool:
        goal_y = self.goal_pose.position.y
        current_y = self.current_pose.position.y

        # rospy.logwarn(f"Y[current, goal] ({current_y:.2f}, {goal_y:.2f}) [m,m]")

        return abs(goal_y - current_y) < self.tol_lin

    def at_goal_yaw(self) -> bool:
        goal_quat = utils.array_from_pose(self.goal_pose)[3:]
        goal_yaw = utils.angle_from_quaternion(goal_quat)
        goal_yaw = utils.wrap_angle(goal_yaw)

        current_quat = utils.array_from_pose(self.current_pose)[3:]
        current_yaw = utils.angle_from_quaternion(current_quat)
        current_yaw = utils.wrap_angle(current_yaw)

        # rospy.logwarn(
        #     f"Yaw[current, goal] ({current_yaw:.2f}, {goal_yaw:.2f}) [rad,rad]"
        # )

        yaw_diff = utils.wrap_angle(goal_yaw - current_yaw)

        return abs(yaw_diff) < self.tol_ang

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

        disp_xy = round(
            np.linalg.norm(current_pose_xy - target_pose_xy) - 0.5, 3
        )
        delta_th = target_pose_th - current_pose_th

        disp_th = round(delta_th, 3)
        disp_th = utils.wrap_angle(disp_th)  # wrapping around [-180,180]

        rospy.logwarn(
            f"Robot at distance {abs(disp_xy):.2f}[m] and {abs(disp_th):.2f}[rad] from target"
        )

        return disp_xy, disp_th

class GiraffeMoveX(Move):
    def __init__(
        self,
        distance: float,
        feedback_pub: Callable[[Pose], None],
        max_lin_vel: float = 0.2,
        tol_lin: float = 0.05,
        reverse: bool = False,
        kp: float = 0.8,
    ):
        super().__init__()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.distance = distance
        self.feedback_pub = feedback_pub
        self.max_lin_vel = max_lin_vel
        self.tol_lin = tol_lin
        self.kp = kp
        self.reverse = reverse

        self.initial_x = None
        self.initial_y = None

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.initial_x == None or self.initial_y == None:
            self.initial_x = self.current_pose.position.x
            self.initial_y = self.current_pose.position.y

    def init_move(self) -> bool:
        """
        Move the robot forward in the x direction until it reaches the goal
        """
        if not self.reverse:
            rospy.loginfo(f"moving forwards {self.distance} m")
        else:
            rospy.loginfo(f"moving backwards {self.distance} m")

        if self.at_goal_x():
            return True

        while not rospy.is_shutdown() and not self.at_goal_x():
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist_x()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot once it reaches the goal
        self.twist_pub.publish(self.compute_empty_twist())
        rospy.loginfo("Success, reached x goal")

        return self.at_goal_x()

    def compute_twist_x(self) -> Twist:
        """
        Computes the twist message to move in the x direction
        """
        twist_msg = Twist()

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_x_local = np.cos(current_yaw) * delta_x_global + np.sin(current_yaw) * delta_y_global
        vel = self.kp * (self.distance - delta_x_local)
        if self.reverse:
            vel = -vel

        twist_msg.linear.x = np.clip(vel, -self.max_lin_vel, self.max_lin_vel)
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0  # No rotation needed

        rospy.loginfo(
            f"MoveX: Sending velocity {twist_msg.linear.x:.2f} [m/s] in x direction"
        )
        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_x(self) -> bool:
        """
        Check if the robot has moved the desired forward distance
        """
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_x_local = np.cos(current_yaw) * delta_x_global + np.sin(current_yaw) * delta_y_global

        if self.reverse:
            target_x = -self.distance
        else:
            target_x = self.distance

        return abs(target_x - delta_x_local) < self.tol_lin

class GiraffeMoveY(Move):
    def __init__(
        self,
        distance: float,
        feedback_pub: Callable[[Pose], None],
        max_lin_vel: float = 0.2,
        tol_lin: float = 0.05,
        left: bool = False,
        kp: float = 0.8,
    ):
        super().__init__()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.distance = distance
        self.feedback_pub = feedback_pub
        self.max_lin_vel = max_lin_vel
        self.tol_lin = tol_lin
        self.kp = kp
        self.left = left

        self.initial_x = None
        self.initial_y = None

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.initial_x == None or self.initial_y == None:
            self.initial_x = self.current_pose.position.x
            self.initial_y = self.current_pose.position.y

    def init_move(self) -> bool:
        """
        Move the robot forward in the x direction until it reaches the goal
        """
        if not self.left:
            rospy.loginfo(f"moving right {self.distance} m")
        else:
            rospy.loginfo(f"moving left {self.distance} m")

        if self.at_goal_y():
            return True

        while not rospy.is_shutdown() and not self.at_goal_y():
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist_y()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot once it reaches the goal
        self.twist_pub.publish(self.compute_empty_twist())
        rospy.loginfo("Success, reached y goal")

        return self.at_goal_y()

    def compute_twist_y(self) -> Twist:
        """
        Computes the twist message to move in the x direction
        """
        twist_msg = Twist()

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_y_local = -np.sin(current_yaw) * delta_x_global + np.cos(current_yaw) * delta_y_global
        vel = self.kp * (self.distance - delta_y_local)
        if not self.left:
            vel = -vel

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = np.clip(vel, -self.max_lin_vel, self.max_lin_vel)
        twist_msg.angular.z = 0.0  # No rotation needed

        rospy.loginfo(
            f"MoveY: Sending velocity {twist_msg.linear.y:.2f} [m/s]"
        )
        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_y(self) -> bool:
        """
        Check if the robot has moved the desired forward distance
        """
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_y_local = -np.sin(current_yaw) * delta_x_global + np.cos(current_yaw) * delta_y_global

        if not self.left:
            target_y = -self.distance
        else:
            target_y = self.distance

        return abs(target_y - delta_y_local) < self.tol_lin

class GiraffeTurnYaw(Move):
    def __init__(
        self,
        angle: float,
        feedback_pub: Callable[[Pose], None],
        max_lin_vel: float = 0.2,
        ccw: bool = False,
        kp: float = 0.8,
    ):
        super().__init__()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.angle = angle
        self.feedback_pub = feedback_pub
        self.max_lin_vel = max_lin_vel
        self.kp = kp
        self.ccw = ccw

        self.initial_yaw = None

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.initial_yaw == None:
            current_quat = self.current_pose.orientation
            self.initial_yaw = utils.angle_from_quaternion([
                current_quat.x,
                current_quat.y,
                current_quat.z,
                current_quat.w
            ])
            self.target_yaw = utils.wrap_angle(self.initial_yaw + self.angle)

    def init_move(self) -> bool:
        """
        Move the robot forward in the x direction until it reaches the goal
        """
        if not self.ccw:
            rospy.loginfo(f"turning clockwise {self.angle:.2f} rad")
        else:
            rospy.loginfo(f"turning counter clockwise {self.angle:2f} rad")

        if self.at_goal_yaw():
            return True

        while not rospy.is_shutdown() and not self.at_goal_yaw():
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist_yaw()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot once it reaches the goal
        self.twist_pub.publish(self.compute_empty_twist())
        rospy.loginfo("Success, reached y goal")

        return self.at_goal_yaw()

    def compute_twist_yaw(self) -> Twist:
        """
        Computes the twist message to turn in the yaw direction
        """
        twist_msg = Twist()

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_yaw = utils.wrap_angle(current_yaw - self.target_yaw)
        rospy.logwarn(f"[target, cur]: [{self.target_yaw:.2f}, {current_yaw:.2f}]")
        rospy.logwarn(f"[delta_yaw]: [{delta_yaw:.2f}]")

        if not self.ccw and delta_yaw > 0: 
            delta_yaw -= 2 * np.pi
        elif self.ccw and delta_yaw < 0:
            delta_yaw += 2 * np.pi  

        vel = self.kp * delta_yaw

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = np.clip(vel, -self.max_ang_vel, self.max_ang_vel)

        rospy.loginfo(
            f"TurnYaw: Sending velocity {twist_msg.angular.z:.2f} [rad/s]"
        )
        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_yaw(self) -> bool:
        """
        Check if the robot has moved the desired forward distance
        """

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])
        
        yaw_diff = utils.wrap_angle(self.target_yaw - current_yaw)

        return abs(yaw_diff) < self.tol_ang

class GiraffePredefinedPath(PredefinedPath):
    """
    """

    def __init__(
        self,
        feedback_pub: Callable[[Pose], None],
        path_type: String,
        rate: float = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        self.path_type: String = path_type
        self.feeback_pub: Callable[[Pose], None] = feedback_pub
        self.current_pose: Optional[Pose] = None
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.tol_lin = tol_lin
        self.tol_ang = tol_ang
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def init_predefined_path(self) -> bool:
        """
        """
        # first check if robot is at start pose from odometry with tolerance
        if self.path_type == String("pothole"):
            task = GiraffePotholeFilling(self.feeback_pub)
            rospy.loginfo("Commencing pot hole path")
            success = task.init_pothole_filling()
        else:
            rospy.logerr("only pothole path is defined!!")
            raise NotImplementedError

        return success


class GiraffePotholeFilling(PotholeFilling):
    def __init__(self, feedback_pub: Callable[[Pose], None]) -> None:

        # first get pothole pose from paramter server
        # then move towards with distance. Save as safe postion
        self.load_parameters()

        self.pothole_sub = rospy.Subscriber(
            "pothole_goal", Pose, self.pothole_cb, queue_size=10
        )
        rospy.sleep(1)  # wait for subscriber to latch
        rospy.logwarn(f"Goal : {self.pothole_goal}")
        self.move_obj = GiraffeMove(self.pothole_goal, feedback_pub)
        rospy.logwarn(f"current pose: {self.move_obj.current_pose}")
        rospy.logwarn(f"goal pose: {self.move_obj.goal_pose}")

        self.safe_pose = utils.find_parallel_pose(
            self.pothole_goal,
            self.move_obj.current_pose,
            self.pothole_diameter * 2,
            towards=True,
        )

        self.roller_up = True

    def pothole_cb(self, msg):
        self.pothole_goal: Pose = msg

    def load_parameters(self) -> None:
        # self.pothole_goal = utils.empty_pose() # initialise empty pose
        self.move_base_dist: float = rospy.get_param("~move_base_dist", 1.0)
        self.deposit_time: float = rospy.get_param("~deposit_time", 5.0)
        self.pothole_diameter: float = rospy.get_param("~pothole_diameter", 0.3)

    def init_pothole_filling(self) -> bool:
        # this fnc should sequence through the diff tasks below
        rospy.logwarn("moving towards pothole location")
        if self.move_to_start_pos():
            self.move_over_pothole()
            # deposit_material
            rospy.logwarn("starting deposit")
            self.deposit_material()
            success = True
        else:
            rospy.logerr("did not get to start")
            success = False

        # moving back to start position
        rospy.logwarn("moving back to start")
        success = self.move_to_start_pos()
        # smooth_pothole
        # rospy.logwarn("starting smoothing of pothole")
        # self.smooth_pothole()

        # TODO need to add some checking here
        return success

    def move_to_start_pos(self) -> bool:
        rospy.loginfo("moving to start position!")

        # if disp_xy >= self.move_base_dist:
        rospy.loginfo("move base to safe position")
        # TODO somehow put this
        move_base_client = MoveBaseClient()
        move_base_client.init_move_base(self.safe_pose)
        while True:
            status = move_base_client.get_status()
            if status == 3:
                self.move_obj.goal_pose = self.safe_pose
                rospy.loginfo("move to safe position")
                success = self.move_obj.init_move_sequence()
                break
            elif status == 0 or status == 1:
                success = False
            else:
                success = False
                rospy.logerr("MoveBase failed to get to safe position")
                break

        return success

    def move_over_pothole(self):
        # set pothole centre as goal
        self.move_obj.goal_pose = self.pothole_goal

        # turn front towards pothole goal
        # self.move_obj.turn_yaw()

        # move only x position
        self.move_obj.move_x()
        return

    def deposit_material(self):
        self.open_hatch()
        rospy.loginfo(f"Depositing material for %f s", self.deposit_time)
        rospy.sleep(self.deposit_time)
        self.close_hatch()

    def open_hatch(self):
        # some srv call that shows a marker on rviz
        rospy.logwarn("opening hatch!")

    def close_hatch(self):
        rospy.logwarn("closing hatch!")

    # need to create waypoint follower

    def find_sweep_points(
        self, pose_a: Pose, pose_b: Pose, dist_1: float, dist_2: float = None
    ) -> list:
        """
        pose_a : start
        pose_b : goal

        finds 3 straight trajectories all parallel to pose_a -> pose_b
        length of trajectories : dist_2 (default is dist_1 * 2)
        
        (P1 -> P2) goes through (pose_a -> pose_b) with pose_b as midpoint
        (P3 -> P4)  & (P5 -> P6) are parallel to (P1 -> P2) with dist_1 / 2

        returns P1, P2, P3, P4, P5, P6 
        """
        # TODO NEED TO CONSIDER ORIENTATION HERE!!!
        if dist_2 == None:
            dist_2 = 2 * dist_1

        point_a = utils.point_from_pose(pose_a)
        point_b = utils.point_from_pose(pose_b)

        para_slope = utils.find_slope(point_a, point_b, perpendicular=False)
        perp_slope = utils.find_slope(point_a, point_b, perpendicular=True)

        # distance for perp line (d)
        if perp_slope == None:  # vertical slope
            d = dist_1 / 2.0
            perp_1 = (point_b[0], point_b[1] + d)
            perp_2 = (point_b[0], point_b[1] - d)

        elif perp_slope == 0:  # horizontal slope
            d = dist_1 / 2.0
            perp_1 = (point_b[0] + d, point_b[1])
            perp_2 = (point_b[0] - d, point_b[1])

        else:
            d = (dist_1 / 2.0) / np.sqrt(1 + perp_slope ** 2)
            perp_1 = (point_b[0] + d, point_b[1] + perp_slope * d)
            perp_2 = (point_b[0] - d, point_b[1] - perp_slope * d)

        # distance for perp line (d)
        d = dist_2 / 2.0
        if para_slope == None:  # vertical slope
            P4 = (perp_1[0], perp_1[1] + d)
            P6 = (perp_2[0], perp_2[1] - d)
            P3 = (perp_1[0], perp_1[1] - d)
            P5 = (perp_2[0], perp_2[1] + d)

        elif perp_slope == 0:  # horizontal slope
            P4 = (perp_1[0] + d, perp_1[1])
            P6 = (perp_2[0] - d, perp_2[1])
            P3 = (perp_1[0] - d, perp_1[1])
            P5 = (perp_2[0] + d, perp_1[1])

        else:
            d = d / np.sqrt(1 + para_slope ** 2)
            P4 = (perp_1[0] + d, perp_1[1] + para_slope * d)
            P6 = (perp_2[0] + d, perp_2[1] + para_slope * d)
            P3 = (perp_1[0] - d, perp_1[1] - para_slope * d)
            P5 = (perp_2[0] - d, perp_2[1] - para_slope * d)

        P1 = utils.find_midpoint(P3, P5)
        P2 = utils.find_midpoint(P4, P6)

        P1 = utils.pose_from_point(P1)
        P2 = utils.pose_from_point(P2)
        P3 = utils.pose_from_point(P3)
        P4 = utils.pose_from_point(P4)
        P5 = utils.pose_from_point(P5)
        P6 = utils.pose_from_point(P6)

        return [P1, P2, P3, P4, P5, P6]

    def calc_smoothing_waypoints(
        self,
        pothole_goal: Pose,
        initial_pose: Pose,
        pothole_diameter: float = 0.3,
    ) -> dict:

        sweep_points = self.find_sweep_points(
            pothole_goal, initial_pose, pothole_diameter * 0.75
        )

        # initial_pose -> P1
        # lower roller
        # P1 -> P2
        # raise roller
        # P2 -> P1
        # P1 -> P3
        # lower roller
        # P3 -> P4
        # raise roller
        # P4 -> P3 (backwards)
        # P3 -> P5zip and a list of tuples if you primarily need indexed access and want to keep it simple.
        # lower roller
        # P5 -> P6
        # raise roller

        ###TODO make this more verbose by writing out zip (pose, true)
        points = [
            initial_pose,
            sweep_points[0],
            sweep_points[1],
            sweep_points[0],
            sweep_points[2],
            sweep_points[3],
            sweep_points[2],
            sweep_points[4],
            sweep_points[5],
        ]
        roller_up = [True, False, True, True, False, True, True, False, True]
        waypoints = list(zip(points, roller_up))
        return waypoints

    def smooth_pothole(self) -> bool:
        rospy.loginfo("Starting smoothing procedure!")
        # first drive back to pos
        self.move_obj.goal_pose = self.safe_pose
        self.move_obj.turn_yaw()
        self.move_obj.move_x()
        #
        waypoints = self.calc_smoothing_waypoints(
            self.pothole_goal, self.safe_pose, self.pothole_diameter
        )
        # waypoints have start position
        for waypoint, move_roller_up in waypoints:
            self.move_obj.goal_pose = waypoint
            self.move_obj.turn_yaw()
            self.move_obj.move_x()

            if move_roller_up != self.roller_up:
                if move_roller_up:
                    self.raise_roller()
                elif not move_roller_up:
                    self.lower_roller()

        if not self.roller_up:
            self.raise_roller()

        return True

    def lower_roller(self):
        rospy.logwarn("lowering roller!")
        self.roller_up = False

    def raise_roller(self):
        rospy.logwarn("raising roller!")
        self.roller_up = True

    def retreat_from_pothole(self):
        # move to safe distance away from pothole
        pass
