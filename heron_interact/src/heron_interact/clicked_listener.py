#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def clicked_point_cb(msg):
    rospy.loginfo("Clicked point recieved: ")
    rospy.loginfo("    x: %.2f, y: %.2f", msg.x, msg.y)

def main():
    rospy.init_node("clicked_listener", anonymous=True)
    rospy.Subscriber("/clicked_point", Point, clicked_point_cb)
    rospy.loginfo("Listening for clicked points...")

    rospy.spin()

if __name__=='__main__':
    main()
