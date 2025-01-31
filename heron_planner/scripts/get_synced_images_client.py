#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image, CameraInfo
from heron_msgs.srv import (
    GetSynchedImages,
    GetSynchedImagesRequest,
    GetSynchedImagesResponse,
)

def req_image_data(camera_ns: str) -> None:
    rospy.wait_for_service("hlp/get_synched_images")
    try:
        get_image_data = rospy.ServiceProxy("hlp/get_synched_images", GetSynchedImages)
        res = get_image_data(camera_ns)
        if res.success:
            rospy.loginfo("Recieved image data")
        else:
            rospy.logwarn("Failed to get image data")
    except rospy.ServiceException as err:
        rospy.logerr(f"Service call failed: {err}")



if __name__ == "__main__":
    rospy.init_node("get_synched_images_client")
    camera_ns = "/robot/body_camera/front_rgbd_camera"
    req_image_data(camera_ns)
