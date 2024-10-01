#!/usr/bin/env python
import os
import subprocess

import rospy
import rospkg
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

import utils.gazebo_utils as utils


def main():

    rospy.init_node("spawn_model")

    model_name = "arrow"
    # model_name = "platypus"
    
    pkg = rospkg.RosPack()
    pkg_path = pkg.get_path('heron_demo')

    model_mesh_path = os.path.join(pkg_path + '/meshes')
    gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH', '')
    
    if model_mesh_path not in gazebo_model_path:
        new_gazebo_model_path = f"{gazebo_model_path}:{model_mesh_path}" if gazebo_model_path else model_mesh_path
        os.environ['GAZEBO_MODEL_PATH'] = new_gazebo_model_path
        print(f"GAZEBO_MODEL_PATH set to: {new_gazebo_model_path}")
    else:
        print(f"GAZEBO_MODEL_PATH already contains: {model_mesh_path}")
    
    model_path = pkg_path + "/urdf/arrow.sdf"
    # model_path = pkg_path + "/urdf/platypus.sdf"
    # model_path = pkg_path + "/meshes/platypus/platypus.sdf"

    up_pose = Pose()
    up_pose.position.x = 0.0
    up_pose.position.y = 0.0
    up_pose.position.z = 0.0
    up_pose.orientation.w = 1.0

    down_pose = Pose()
    down_pose.position.x = 0.0
    down_pose.position.y = 0.0
    down_pose.position.z = 0.54
    down_pose.orientation.x = 1.0

    success = utils.spawn_model(model_name, model_path, up_pose)
    if success:
        rospy.loginfo("Model spawned successfully")
    else:
        rospy.logerr("Failed to spawn model")

    rospy.sleep(2.0)
    rospy.loginfo("teleporting upsidedown")
    uccess = utils.teleport_model(model_name, down_pose)
    rospy.sleep(2.0)
    rospy.loginfo("teleporting upsidedown")
    success = utils.teleport_model(model_name, up_pose)
    rospy.sleep(2.0)
    rospy.loginfo("deleting")
    success = utils.delete_model(model_name)



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
