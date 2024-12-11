#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node("test_image_pub")

    rospy.wait_for_service("/kafka/send_body_image")
    try:
        rospy.loginfo("found srv")
        srv = rospy.ServiceProxy("/kafka/send_body_image", Empty)

    except rospy.ServiceException as err:
        rospy.logerr(f"Service call to /kafka/send_body_image failed : {err}")


if __name__ == "__main__":
    main()
