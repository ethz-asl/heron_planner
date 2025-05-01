#!/usr/bin/env python

import rospy
import tf2_ros
import math
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


class CarrotTFPublisher:
    def __init__(self):
        rospy.init_node('carrot_tf_publisher')

        self.speed = rospy.get_param('~speed', 0.1)  # meters per second
        self.path_sub = rospy.Subscriber('/hlp/path', Path, self.path_callback)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.start = None
        self.end = None
        self.duration = 0.0
        self.start_time = None
        self.yaw = 0.0
        self.finished = True


        self.timer = rospy.Timer(rospy.Duration(0.01), self.publish_tf)  # 10 Hz

    def path_callback(self, msg):
        if len(msg.poses) != 2:
            rospy.logwarn("Path must contain exactly 2 poses.")
            return

        self.start = msg.poses[0].pose
        self.end = msg.poses[1].pose

        dx = self.end.position.x - self.start.position.x
        dy = self.end.position.y - self.start.position.y
        dz = self.end.position.z - self.start.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        self.duration = distance / self.speed if self.speed > 0 else 0
        self.start_time = rospy.Time.now()

        # Compute yaw angle from start to end
        self.yaw = math.atan2(dy, dx)

        rospy.loginfo("Received path. Duration: {:.2f}s, Yaw: {:.2f} rad, Start: ({:.2f}, {:.2f}), End: ({:.2f}, {:.2f})".format(self.duration, self.yaw, self.start.position.x, self.start.position.y, self.end.position.x, self.end.position.y))
        self.finished = False

    def publish_current_tf(self):
        try:
            # Lookup the transform from robot_odom to robot_base_frame
            transform = self.tf_buffer.lookup_transform(
                'robot_odom',           # target_frame (parent)
                'robot_base_footprint',     # source_frame (child)
                rospy.Time(0),          # get latest available
                rospy.Duration(1.0)     # timeout
            )

            # Modify the transform to use "carrot" as the child frame
            new_transform = geometry_msgs.msg.TransformStamped()
            new_transform.header.stamp = rospy.Time.now()
            new_transform.header.frame_id = "robot_odom"   # parent
            new_transform.child_frame_id = "carrot"        # new child
            new_transform.transform = transform.transform

            # Broadcast the new transform
            self.br.sendTransform(new_transform)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5.0, "TF2 lookup failed. Retrying...")

    def publish_tf(self, event):
        if not self.start or not self.end or self.duration <= 0 or not self.start_time:
            self.publish_current_tf()
            return
        
        seconds_to_wait = 5.0

        elapsed = max((rospy.Time.now() - self.start_time).to_sec() - 5.0, 0.0)
        ratio = min(elapsed / self.duration, 1.0)

        # Linear interpolation
        x = (1 - ratio) * self.start.position.x + ratio * self.end.position.x
        y = (1 - ratio) * self.start.position.y + ratio * self.end.position.y
        z = (1 - ratio) * self.start.position.z + ratio * self.end.position.z

        # Use yaw to get orientation quaternion
        q = quaternion_from_euler(0, 0, self.yaw)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "robot_odom"
        t.child_frame_id = "carrot"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)
        if not self.finished:
            rospy.loginfo_throttle(1, f"Progress: {ratio*100:0.2f}%, Position: {x:.2f}, {y:.2f}")

        if ratio >= 1.0 and not self.finished:
            self.finished = True
            rospy.loginfo("Finished.")

if __name__ == '__main__':
    try:
        CarrotTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
