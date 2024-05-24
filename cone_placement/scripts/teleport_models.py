#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose


def teleport_model(model_name: str, new_pose: Pose) -> bool:
    """
    teleport named model in gazebo to new pose
    """
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = new_pose
        res = service(model_state)
        return res.success
    except rospy.ServiceException as err:
        rospy.logerr(f"Service call failed: {err}")
        return False


def main():
    rospy.init_node("teleport_cone")

    model_name = "cone_3"
    new_pose = Pose()
    new_pose.position.x = 2.0
    new_pose.position.y = 3.0
    new_pose.position.z = 0.0
    new_pose.orientation.w = 1.0

    success = teleport_model(model_name, new_pose)
    if success:
        rospy.loginfo("Model teleported successfully")
    else:
        rospy.logerr("Failed to teleport model")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
