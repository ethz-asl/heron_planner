#!/usr/bin/env python

import rospy
import py_trees as pt
from typing import List, Any
import threading
import numpy as np

from std_msgs.msg import Empty, String, Float64
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from heron_msgs.srv import (
    FindPothole,
    FindPotholeRequest,
    FindPotholeResponse,
    FindCrack,
    FindLaneEdge,
)


class RetryUntilSuccessful(pt.decorators.Decorator):
    """
    custom decorator that retries its child until it suceeds 
    or a maximum num of attempts has been reached
    """
    def __init__(self, child, max_attempts, name="RetryUntilSuccessful"):
        super().__init__(name=name, child=child)
        self.child = child
        self.max_attempts = max_attempts
        self.current_attempt = 1

    def initialise(self):
        """reset the attempt counter"""
        pass

    def update(self) -> pt.common.Status:
        if self.current_attempt >= self.max_attempts:
            rospy.logwarn(f"Max attempts ({self.max_attempts}) reached.")
            return pt.common.Status.FAILURE
        
        child_status = self.child.status
        
        if child_status == pt.common.Status.SUCCESS:
            rospy.loginfo(f"Child succeeded")
            return pt.common.Status.SUCCESS
        elif child_status == pt.common.Status.FAILURE:
            self.current_attempt += 1
            rospy.logerr(f"Retrying... Attempt ({self.current_attempt} / {self.max_attempts})")
            # self.child.stop(pt.common.Status.INVALID) # reset child
            return pt.common.Status.RUNNING
        else:
            rospy.logerr(f"Retrying... waiting ({self.current_attempt} / {self.max_attempts})")
            return pt.common.Status.RUNNING
        

class GetLocationFromQueue(pt.behaviour.Behaviour):
    """
    subscribe to queue topic and

    @param queue_topic (str)
    @param current_item_topic (str)

    """

    def __init__(self, name, locations_param, location_topic: str):
        super().__init__(name)
        self.locations_param = locations_param
        self.location_pub = rospy.Publisher(location_topic, String, queue_size=10)
        self.queue = rospy.get_param(self.locations_param, [])

    def initialise(self):
        rospy.logerr(f"self.queue {self.queue}")

    def update(self):

        if not self.queue:
            rospy.logerr(f"No more locations in the queue")
            return pt.common.Status.FAILURE
        
        target_location_msg = String(data=self.queue.pop(0))
        self.location_pub.publish(target_location_msg)
        rospy.loginfo(f"Next location: {target_location_msg.data}")
        return pt.common.Status.SUCCESS


