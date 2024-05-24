#! /usr/bin/env python

import rospy
import copy
import heron_planner.cone_placement.scripts.heron_markers as vis
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import numpy as np


def main():

    rospy.init_node("marker_node")
    cones = vis.DrawMarkers()

    P1 = Pose(Point(3, -1, 0), Quaternion(0, 0, 0, 1))
    P2 = Pose(Point(2, -1, 0), Quaternion(0, 0, 0, 1))
    P3 = Pose(Point(2, 0, 0), Quaternion(0, 0, 0, 1))
    P4 = Pose(Point(3, 0, 0), Quaternion(0, 0, 0, 1))

    poses = np.array([P1, P4])
    colours = np.array(["orange", "teal"])
    ids = np.array([0, 1])

    cones.clear()
    count = 0
    while not rospy.is_shutdown():

        p = copy.deepcopy(poses)
        c = copy.deepcopy(colours)
        i = copy.deepcopy(ids)
        cones.draw_cones(p, c, i)
        p = copy.deepcopy([P2])
        c = ["light_purple"]
        cones.draw_cones(p, c, [2])
        rospy.rostime.wallsleep(1.0)
        cones.clear_cone(1)
        cones.clear_cone(2)
        rospy.rostime.wallsleep(1.0)
        cones.show_cone(1)
        rospy.rostime.wallsleep(0.5)
        if count == 5:
            break
        count += 1
        cones.clear()

    rospy.logwarn("clearing grasps")
    cones.clear()
    rospy.rostime.wallsleep(1.0)

    count = 0
    while not rospy.is_shutdown():
        p = copy.deepcopy([P4])
        c = ["purple"]
        cones.draw_cones(p, c, [3])
        rospy.rostime.wallsleep(2.0)
        cones.clear()
        if count == 5:
            break
        count += 1

    rospy.logwarn("clearing grasps")
    cones.clear()


if __name__ == "__main__":
    main()
