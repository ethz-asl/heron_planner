#! /usr/bin/env python

import rospy
import copy
from heron_markers import DrawCones
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from gazebo_msgs.srv import GetModelState
import numpy as np


class ConeMarkerNode:
    def __init__(self):
        self.load_parameters()
        self.cones = DrawCones()
        self.cones.clear()

    def load_parameters(self):
        self.model_names = rospy.get_param("heron_demo/gazebo_cones")

    def get_model_pose(self, model_name: str) -> Pose:
        """
        get model pose from name in gazebo
        """
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            srv = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
            res = srv(model_name, "")
            return res.pose
        except rospy.ServiceException as err:
            rospy.logerr(f"Service call to /gazebo/get_model failed: {err}")
            return None

    def draw_cone_markers(self):
        n_cones = len(self.model_names)
        self.poses = [Pose() for _ in range(n_cones)]
        self.colours = np.full(n_cones, "orange")
        self.indexes = np.arange(n_cones)

        for idx, model_name in enumerate(self.model_names):
            self.poses[idx] = self.get_model_pose(model_name)

        poses = copy.deepcopy(self.poses)
        cols = copy.deepcopy(self.colours)
        ids = copy.deepcopy(self.indexes)

        self.cones.draw_cones(poses, cols, ids)


def main():

    rospy.init_node("cone_marker_node")

    rate = rospy.Rate(1)
    cone = ConeMarkerNode()

    while not rospy.is_shutdown():
        cone.draw_cone_markers()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
