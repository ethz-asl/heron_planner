#!/usr/bin/env python

from __future__ import annotations  # for type hinting
from typing import Callable

import os
import rospy
import rospkg
import numpy as np

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from robot_interface.base_robot_interface import (
    Move,
    AtPose,
    ParamClient,
    TopicClient,
    ComponentStatus,
)

import heron_utils.gazebo_utils as gazebo_utils
import heron_utils.transform_utils as utils

class UgvCaptureImage():
    def __init__(self, ):
        """
        capture photo to camera and send to kafka
        """
        




class UgvParamClient(ParamClient):
    def __init__(self, param_name: str, check_condition: callable):
        self._param_name = param_name
        self._check_condition = check_condition

    def is_condition_met(self):
        param_val = rospy.get_param(self._param_name, None)
        return self._check_condition(param_val)


class UgvTopicClient(TopicClient):
    def __init__(self, topic_name: str, msg_type, check_condition: callable):
        self._is_condition_met = False
        self._check_condition = check_condition
        rospy.Subscriber(topic_name, msg_type, self._topic_cb)

    def _topic_cb(self, msg):
        self._is_condition_met = self._check_condition(msg)

    def is_condition_met(self):
        return self._is_condition_met
