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
        goal_pose: Pose = Pose(),
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


class PredefinedPath(ABC):
    def __init__(
        self,
        path_type: str = "pothole",
        rate: float = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        self.path_type: str = path_type
        self.current_pose: Optional[Pose] = None
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.tol_lin = tol_lin
        self.tol_ang = tol_ang
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel

    @abstractmethod
    def init_predefined_path(self) -> bool:
        pass 

class PotholeFilling(ABC):
    def __init__(
        self,
        rate: float = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.tol_lin = tol_lin
        self.tol_ang = tol_ang
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel

    @abstractmethod
    def init_pothole_filling(self) -> bool:
        pass

    @abstractmethod
    def move_over_pothole(self):
        """
        move linear x to pothole position slowly
        """
        pass

    @abstractmethod
    def deposit_material(self):
        """
        open hatch
        wait x seconds
        close hatch
        """
        pass
    
    @abstractmethod
    def open_hatch(self):
        pass

    @abstractmethod
    def close_hatch(self):
        pass
    
    @abstractmethod
    def smooth_pothole(self):
        """        
        move to original start position
        drop roller
        move in 3 sweeps over pothole location - predefined path
        raise roller
        """
        pass

    @abstractmethod
    def lower_roller(self):
        pass

    @abstractmethod
    def raise_roller(self):
        pass

    @abstractmethod
    def retreat_from_pothole(self):
        pass