#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import scipy as sc
from abc import ABC, abstractmethod
from typing import Optional
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus



class Move(ABC):
    def __init__(
        self,
        goal_pose: Pose,
        rate: float = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        self.goal_pose: Pose = goal_pose
        self.current_pose: Optional[Pose] = None
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.tol_lin = tol_lin
        self.tol_ang = tol_ang
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        #  self.odom_sub = rospy

    @abstractmethod
    def odom_cb(self, msg: Odometry) -> None:
        pass

    @abstractmethod
    def init_move(self) -> None:
        pass

    @abstractmethod
    def at_goal_x(self) -> bool:
        pass

    @abstractmethod
    def at_goal_y(self) -> bool:
        pass

    @abstractmethod
    def at_goal_yaw(self) -> bool:
        pass
    
    @abstractmethod
    def _move_x(self) -> None:
        """Move robot in X direction for predefined distance"""
        pass

    @abstractmethod
    def _move_y(self) -> None:
        """Move robot in X direction for predefined distance"""
        pass
    
    @abstractmethod
    def _turn_yaw(self) -> None:
        """Move robot in Z angular direction for predefined distance"""
        pass
