#!/usr/bin/env python3

import rospy
import ros_trees as rt

from geometry_msgs.msg import PoseStamped

################################################################################
############################# parameters from config ###############################
################################################################################

FIND_POTHOLE_SRV = rospy.get_param(
    "iccs/find_pothole_srv", "/iccs/find_pothole"
)

################################################################################
############################### Leaf definitions ###############################
################################################################################


class FindPothole(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(FindPothole, self).__init__(
            name=task_name if task_name else "Find pothole",
            service_name=FIND_POTHOLE_SRV,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )

    def _result_fn(self):
        res = self._default_result_fn()
        rospy.logwarn(f"Found pothole? : {res.success}")
        if not res.success:
            return False

        if isinstance(res.center_of_mass, PoseStamped) and isinstance(
            res.surface_area_m, float
        ):
            rt.data_management.set_value("/pothole/com", res.center_of_mass)
            rt.data_management.set_value(
                "/pothole/surface_area", res.surface_area_m
            )
            return res.success
        else:
            rospy.logwarn(f"Response incorrect type.")
