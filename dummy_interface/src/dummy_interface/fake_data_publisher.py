#!/usr/bin/env python

import rospy
from robotnik_msgs.msg import BatteryStatus, State
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Header
import random
import numpy as np
from cv_bridge import CvBridge


class FakeDataPublisher:
    def __init__(self):
        rospy.init_node("fake_data_publisher")

        # Publishers
        self.battery_pub = rospy.Publisher(
            "/robot/battery_estimator/data", BatteryStatus, queue_size=10
        )
        self.gps_pub = rospy.Publisher(
            "/robot/gps/fix", NavSatFix, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/robot/robot_local_control/RobotStatusComponent/state",
            State,
            queue_size=10,
        )
        self.arm_camera_pub = rospy.Publisher(
            "/robot/arm_camera/front_rgbd_camera/rgb/image_raw",
            Image,
            queue_size=10,
        )
        self.body_camera_pub = rospy.Publisher(
            "/robot/body_camera/front_rgbd_camera/rgb/image_raw",
            Image,
            queue_size=10,
        )

        self.bridge = CvBridge()

        rospy.loginfo("Fake data publisher initialized.")

    def publish_battery_status(self):
        """Publishes a faked battery status."""
        battery_msg = BatteryStatus()
        battery_msg.voltage = random.uniform(12.0, 14.4)  # Voltage in volts
        battery_msg.current = random.uniform(-1.0, 1.0)  # Current in amperes
        battery_msg.level = random.uniform(
            0.0, 100.0
        )  # Battery level in percentage
        battery_msg.time_remaining = random.randint(
            0, 300
        )  # Time remaining in minutes
        battery_msg.time_charging = random.randint(
            0, 60
        )  # Time charging in minutes
        battery_msg.is_charging = random.choice(
            [True, False]
        )  # Charging status
        battery_msg.cell_voltages = [
            random.uniform(3.6, 4.2) for _ in range(6)
        ]  # Cell voltages
        self.battery_pub.publish(battery_msg)
        rospy.loginfo(f"Published fake battery status: {battery_msg}")

    def publish_gps_data(self):
        """Publishes faked GPS coordinates."""
        gps_msg = NavSatFix()
        gps_msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        gps_msg.latitude = random.uniform(-90.0, 90.0)  # Random latitude
        gps_msg.longitude = random.uniform(-180.0, 180.0)  # Random longitude
        gps_msg.altitude = random.uniform(0.0, 1000.0)  # Altitude in meters
        self.gps_pub.publish(gps_msg)
        rospy.loginfo(f"Published fake GPS data: {gps_msg}")

    def publish_robot_status(self):
        """Publishes faked robot status."""
        state_msg = State()
        state_msg.state = random.choice(
            [
                State.INIT_STATE,
                State.STANDBY_STATE,
                State.READY_STATE,
                State.EMERGENCY_STATE,
                State.FAILURE_STATE,
                State.SHUTDOWN_STATE,
                State.UNKOWN_STATE,
            ]
        )  # Random state
        state_msg.desired_freq = 50.0  # Desired control frequency
        state_msg.real_freq = random.uniform(
            45.0, 50.0
        )  # Real control frequency
        state_msg.state_description = "faked state description for testing"
        self.status_pub.publish(state_msg)
        rospy.loginfo(f"Published fake robot status: {state_msg}")

    def publish_camera_image(self, pub):
        """Publishes a faked camera image."""
        # Create a random image
        image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        ros_image.header = Header(
            stamp=rospy.Time.now(), frame_id="camera_link"
        )
        pub.publish(ros_image)
        rospy.loginfo("Published a fake camera image.")

    def run(self):
        rate = rospy.Rate(0.1)  # Publish at 0.1 Hz
        while not rospy.is_shutdown():
            self.publish_battery_status()
            self.publish_gps_data()
            self.publish_robot_status()
            self.publish_camera_image(self.arm_camera_pub)
            self.publish_camera_image(self.body_camera_pub)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = FakeDataPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("fake data publisher shutting down.")
