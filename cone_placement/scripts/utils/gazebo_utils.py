#! /usr/bin/env python

from __future__ import annotations

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

"""
Here make a list of functions that can move objects around in gazebo

"""

def attach_req():
    pass

def dettach_req():
    pass

def move_object():
    """
    should move object in gazebo using 
    publisher: /gazebo/set_model_states
    only needs to publish for 1 sec and then stop
    """
    pass    

def clear_object():
    pass

if __name__=='__main__':
    rospy.init_node("link_attacher")
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    attach_srv.wait_for_service()


    