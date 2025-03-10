#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import cv_bridge
import numpy as np

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Path

class CrackPathGenerator:
    def __init__(self) -> None:
       """
       """ 
       rospy.init_node("crack_path_generator", anonymous=True)

       self.tf_buffer = tf2_ros.Buffer()
       self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

       self.path_pub = rospy.Publisher("hlp/crack_path", Path, queue_size=10)

       # either from bb or subscriber
       rospy.Subscriber("hlp/crack_start", PointStamped, self.start_point_cb)
       rospy.Subscriber("hlp/crack_end", PointStamped, self.end_point_cb)
       rospy.Subscriber("hlp/crack_segmentation", Image, self.segmentation_cb)

       self.bridge = cv_bridge.CvBridge()
       self.start_point = None
       self.end_point = None
       self.seg_mask = None

    def start_point_cb(self, msg: PointStamped) -> None:
        self.start_point = msg
        self.generate_path()

    def end_point_cb(self, msg: PointStamped) -> None:
        self.end_point = msg
        self.generate_path()

    def segmentation_cb(self, msg: Image) -> None:
        self.seg_mask = msg
        self.generate_path()

    def transform_point(self, point: PointStamped, target_frame: str = "odom") -> PointStamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, point.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
            )
            return tf2_geometry_msgs.do_transform_point(point, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(f"TF Transform error: {err}")
            return None
    
    def generate_path(self):
        if self.start_point is None or self.end_point is None or self.seg_mask is None:
            return
        
        rospy.loginfo("transforming points into robot frame")
        start = self.transform_point(self.start_point)
        end = self.transform_point(self.end_point)

        if start is None or end is None:
            return

        rospy.loginfo("found transforms")        
        mask = self.bridge.imgmsg_to_cv2(self.seg_mask, desired_encoding='mono8')
        rospy.loginfo("found encoding in cv2")        
        
        skeleton = cv2.ximgproc.thinning(mask) # extract mask skeleton
        contours, _ = cv2.findContours(skeleton, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if not contours:
            rospy.logwarn("No crack centreline found")
            return
        
        # select longest contour as crack path
        rospy.loginfo("finding longest contour")        
        contour = max(contours, key=len)
        points = np.array([point[0] for point in contour])

        # convert pixel to realworld coords
        rospy.loginfo("converting to path")        
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0] * 0.001 # scale to m
            pose.pose.position.y = point[1] * 0.001 
            pose.pose.position.z = 0 # road assumed 2d
            path.poses.append(pose)

        self.path_pub.publish(path)
        rospy.loginfo("Published crack path")

if __name__ == "__main__":
    try:
        CrackPathGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
