#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Path

def publish_test_data():
    rospy.init_node('test_crack_path_publisher', anonymous=True)
    
    start_pub = rospy.Publisher("hlp/crack_start", PointStamped, queue_size=10)
    end_pub = rospy.Publisher("hlp/crack_end", PointStamped, queue_size=10)
    mask_pub = rospy.Publisher("hlp/crack_segmentation", Image, queue_size=10)
    overlay_pub = rospy.Publisher("hlp/crack_overlay", Image, queue_size=10)

    bridge = CvBridge()
    rate = rospy.Rate(1)

    def path_cb(msg):
        rospy.loginfo("Received crack path")
        
        mask = np.zeros((200, 200, 3), dtype=np.uint8)
        cv2.line(mask, (50, 50), (150, 150), (255, 255, 255), 2)
        
        # Draw the received path
        for pose in msg.poses:
            x = int(pose.pose.position.x * 1000)  # Scale back to pixel space
            y = int(pose.pose.position.y * 1000)
            cv2.circle(mask, (x, y), 2, (0, 255, 0), -1)
        
        # Convert and publish the overlayed image
        overlay_msg = bridge.cv2_to_imgmsg(mask, encoding='bgr8')
        overlay_msg.header.stamp = rospy.Time.now()
        overlay_pub.publish(overlay_msg)

        overlay_pub.publish(mask_msg)

    rospy.Subscriber("/hlp/crack_path", Path, path_cb)

    while not rospy.is_shutdown():
        # Create a fake segmentation mask with a simple crack
        mask = np.zeros((200, 200), dtype=np.uint8)
        cv2.line(mask, (50, 50), (150, 150), 255, 2)
        
        # Convert mask to ROS image
        mask_msg = bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header.stamp = rospy.Time.now()
        mask_msg.header.frame_id = "arm_camera_link"
        
        # Create fake start and end points
        start_point = PointStamped()
        start_point.header.stamp = rospy.Time.now()
        start_point.header.frame_id = "arm_camera_link"
        start_point.point.x = 0.1
        start_point.point.y = 0.1
        start_point.point.z = 0.5
        
        end_point = PointStamped()
        end_point.header.stamp = rospy.Time.now()
        end_point.header.frame_id = "arm_camera_link"
        end_point.point.x = 0.5
        end_point.point.y = 0.5
        end_point.point.z = 1.0
        
        # Publish test data
        start_pub.publish(start_point)
        end_pub.publish(end_point)
        mask_pub.publish(mask_msg)
        
        rospy.loginfo("Published test crack path data")
        rate.sleep()



if __name__ == "__main__":
    try:
        publish_test_data()
    except rospy.ROSInterruptException:
        pass



