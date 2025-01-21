#!/usr/bin/env python

import rospy
from robotnik_msgs.msg import BatteryStatus, State
from sensor_msgs.msg import NavSatFix, Image, NavSatStatus
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf_conversions
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
        self.ee_pose_pub = rospy.Publisher(
            "/robot/arm/ee_pose", 
            PoseStamped,
            queue_size=10
        )
        self.ee_pose_name_pub = rospy.Publisher(
            "/robot/arm/ee_pose_name", 
            String,
            queue_size=10
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
        self.hlp_state_pub = rospy.Publisher("/hlp/state", String, queue_size=10)

        # create transforms for TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.transforms = [
            self.create_transform("odom", "base_link", 0, 0, 0),
            self.create_transform("base_link", "base_camera_link", 0.2, 0.0, 0.5),
            self.create_transform("base_camera_link", "arm_camera_link", 0.1, 0.0, 0.2),
        ]


        self.bridge = CvBridge()

        rospy.loginfo("Fake data publisher initialized.")

    
    def create_transform(self, parent_frame, child_frame, x, y, z, roll=0, pitch=0, yaw=0):
        """
        create a TransformStamped object for broadcasting.
        #TODO move to utils
        """
        transform = TransformStamped()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.header.stamp = rospy.Time.now()

        # set translation
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # set rotation (quaternion)
        quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        return transform

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
        gps_msg.latitude = round(random.uniform(-90.0, 90.0), 6)
        gps_msg.longitude = round(random.uniform(-180.0, 180.0), 6)
        gps_msg.altitude = round(random.uniform(-50.0, 8848.0), 2)  # Altitude in meters
        gps_msg.position_covariance = [0.5] * 9  # Fake fixed covariance matrix
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        gps_msg.status.status = NavSatStatus.STATUS_FIX  # Assume a valid fix
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        self.gps_pub.publish(gps_msg)
        rospy.loginfo(f"Published fake GPS data: {gps_msg}")

    def publish_consistent_gps_data(self):
        """Publishes faked GPS coordinates."""
        gps_msg = NavSatFix()
        gps_msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        gps_msg.latitude = 35 # Random latitude
        gps_msg.longitude = -125  # Random longitude
        gps_msg.altitude = 0  # Altitude in meters
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

    def publish_ee_pose(self):
        """Publishes fake """

        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 0.0

        self.ee_pose_pub.publish(pose_msg)

    def publish_ee_pose_name(self):
        """Publishes fake """

        name_msg = String()
        name_msg.data = "NONE"

        self.ee_pose_name_pub.publish(name_msg)

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

    def publish_hlp_state(self):
        """
        Generate a fake high-level planning (HLP) state as a string.
        """
        hlp_states = [
            "[-] Before inspection [*]\n[o] PotholeOffsetSelector [✓]",
            "[o] HomeSelector [*] --> Arm at home? [✕]",
            "[o] Inspect and find pothole\n-^- Retry Inspection Locations",
            "[o] Search Inspection Location [-] --> Find pothole",
            "[o] No Pothole Found --> Send pothole segmentation to Kafka",
        ]
        hlp_msg = String(data=(random.choice(hlp_states)))
        self.hlp_state_pub.publish(hlp_msg)
        rospy.loginfo("Published a fake hlp state.")

    def publish_transforms(self):
        """
        publish static transforms to the tf tree.
        """
        for transform in self.transforms:
            transform.header.stamp = rospy.Time.now()  
            self.tf_broadcaster.sendTransform(transform)

        rospy.loginfo("Publsihed fake transforms")

    def run(self):
        rate = rospy.Rate(0.1)  # Publish at 0.1 Hz
        while not rospy.is_shutdown():
            self.publish_battery_status()
            self.publish_gps_data()
            # self.publish_consistent_gps_data()
            self.publish_robot_status()
            self.publish_hlp_state()
            self.publish_transforms()
            # self.publish_ee_pose()
            # self.publish_ee_pose_name()
            # self.publish_camera_image(self.arm_camera_pub)
            # self.publish_camera_image(self.body_camera_pub)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = FakeDataPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("fake data publisher shutting down.")
