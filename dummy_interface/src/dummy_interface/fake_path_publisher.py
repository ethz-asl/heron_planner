#!/usr/bin/env python

import rospy
from robotnik_msgs.msg import BatteryStatus, State
from sensor_msgs.msg import NavSatFix, Image, NavSatStatus
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
import tf2_ros
import tf_conversions
import random
import numpy as np
from cv_bridge import CvBridge

BASE_FRAME = rospy.get_param("/ugv/base_frame", "robot_base_footprint")

class FakePathPublisher:
    def __init__(self):
        rospy.init_node("fake_path_publisher")

        # Publishers
        self.path_pub = rospy.Publisher("hlp/crack_path", Path, queue_size=1)
        self.start_pub = rospy.Publisher("hlp/crack_start", PoseStamped, queue_size=10)
        self.end_pub = rospy.Publisher("hlp/crack_end", PoseStamped, queue_size=10)
        self.mid_pub = rospy.Publisher("hlp/crack_mid", PoseStamped, queue_size=10)

        self.path_srv = rospy.Service("hlp/generate_simple_path", GetPlan, self.handle_path)

        rospy.loginfo("Fake path publisher initialized.")

    def handle_path(self, req: GetPlanRequest) -> GetPlanResponse:
        
        res = GetPlanResponse()
        res.plan = Path()
        res.plan.header.frame_id = BASE_FRAME
        res.plan.header.stamp = rospy.Time.now()

        x_pts = np.linspace(req.start.pose.position.x, req.goal.pose.position.x, 11)     
        y_pts = np.linspace(req.start.pose.position.y, req.goal.pose.position.y, 11)

        path_pts = np.column_stack((x_pts, y_pts))

        for x, y in path_pts:
            pose = PoseStamped()
            pose.header = res.plan.header  
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1

            res.plan.poses.append(pose)

        self.path_pub.publish(res.plan)
        self.start_pub.publish(res.plan.poses[0])         
        self.end_pub.publish(res.plan.poses[-1])
        self.mid_pub.publish(res.plan.poses[len(res.plan.poses)//2])         
        rospy.loginfo(f"published easy path!")

        return res
    
    def publish_easy_path(self):
        """create an easy path"""
        path = Path()
        path.header.frame_id = BASE_FRAME
        path.header.stamp = rospy.Time.now()

        x_pts = np.linspace(-0.2, 0.2, 11)
        y_pts = np.ones_like(x_pts) * 0.5

        path_pts = np.column_stack((x_pts, y_pts))
                
        for x, y in path_pts:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1

            path.poses.append(pose)
        
        self.path_pub.publish(path)
        self.start_pub.publish(path.poses[0])
        self.end_pub.publish(path.poses[-1])

        self.mid_pub.publish(path.poses[len(path.poses)//2])
        rospy.loginfo(f"published easy path!")

    def run(self):
        rate = rospy.Rate(0.2)  # Publish at 0.1 Hz
        while not rospy.is_shutdown():
            self.publish_easy_path()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = FakePathPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("fake data publisher shutting down.")
