#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class FakeCameraPublisher:
    def __init__(self, camera_ns="/robot/body_camera/front_rgbd_camera"):
        rospy.init_node("fake_camera_publisher")
        self.bridge = CvBridge()
        self.camera_ns = camera_ns.rstrip("/")  # Normalize namespace

        # Define topic names
        self.rgb_topic = f"{self.camera_ns}/rgb/image_raw"
        self.depth_topic = f"{self.camera_ns}/depth/image_raw"
        self.info_topic = f"{self.camera_ns}/color/camera_info"

        # Publishers
        self.rgb_pub = rospy.Publisher(self.rgb_topic, Image, queue_size=1)
        self.depth_pub = rospy.Publisher(self.depth_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(self.info_topic, CameraInfo, queue_size=1)

        rospy.loginfo(f"Publishing fake camera data on {self.camera_ns} topics...")
        self.publish_fake_data()

    def publish_fake_data(self):
        rate = rospy.Rate(5)  # 5 Hz
        while not rospy.is_shutdown():
            # Create a fake RGB image (blue rectangle)
            rgb_img = np.zeros((480, 640, 3), dtype=np.uint8)
            rgb_img[:, :, 2] = 255  # Blue color

            # Create a fake depth image (grayscale gradient)
            depth_img = np.tile(np.linspace(0, 255, 640, dtype=np.uint8), (480, 1))

            # Create CameraInfo message
            cam_info = CameraInfo()
            cam_info.width = 640
            cam_info.height = 480
            cam_info.distortion_model = "plumb_bob"
            cam_info.K = [525.0, 0.0, 320.0,  # fx, 0, cx
                          0.0, 525.0, 240.0,  # 0, fy, cy
                          0.0, 0.0, 1.0]  # 0, 0, 1
            cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

            # Convert images to ROS Image messages
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="mono8")

            # Publish messages
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(cam_info)

            rospy.loginfo("Published fake RGB, depth, and camera info.")
            rate.sleep()

if __name__ == "__main__":
    try:
        FakeCameraPublisher()
    except rospy.ROSInterruptException:
        pass
