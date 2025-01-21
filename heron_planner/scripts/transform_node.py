#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from heron_msgs.srv import (
    TransformPose, 
    TransformPoseRequest, 
    TransformPoseResponse
)

import tf2_ros
import tf2_geometry_msgs

class TransformFinder:
    def __init__(self):
        rospy.init_node('transform_finder')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.service = rospy.Service('transform_pose', TransformPose, self.handle_transform_req)
        rospy.loginfo("transform pose service is ready.")

    def handle_transform_req(self, req: TransformPoseRequest):
        try:
            target_frame = req.target_frame
            pose_in = req.pose_in
            rospy.logwarn(f"current frame: {pose_in.header.frame_id}, target frame: {target_frame}")

            # lookup transform from the input pose frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_in.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)  # wait time for the transform
            )

            pose_out = tf2_geometry_msgs.do_transform_pose(pose_in, transform)
            pose_out.header.stamp = pose_in.header.stamp # syncing timestamp

            return TransformPoseResponse(
                success=True,
                pose_out=pose_out
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {str(e)}")
            return TransformPoseResponse(
                success=False,
            )

if __name__ == '__main__':
    try:
        TransformFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
