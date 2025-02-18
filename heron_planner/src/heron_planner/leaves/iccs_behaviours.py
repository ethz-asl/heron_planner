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
    def __init__(self, task_name="", save_bb_key="", *args, **kwargs) -> None:
        super(FindPothole, self).__init__(
            name=task_name if task_name else "Find pothole",
            service_name=FIND_POTHOLE_SRV,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.save_bb_key = save_bb_key
        rospy.logerr(f"save_bb_key : {save_bb_key}")

    def _result_fn(self):
        com = "/pothole/com"
        surface_area = "/pothole/surface_area"
        success_key = "/pothole/success"

        if self.save_bb_key is not None:
            bb_key = rt.data_management.get_value(self.save_bb_key)
            com = com + "/" + bb_key
            surface_area = surface_area + "/" + bb_key
            success_key = success_key + "/" + bb_key
 
        res = self._default_result_fn()
        rospy.logwarn(f"Found pothole? : {res.success}")
        rt.data_management.set_value(success_key, res.success)

        if not res.success:
            return False

        if isinstance(res.center_of_mass, PoseStamped) and isinstance(
            res.surface_area_m, float
        ):

            rt.data_management.set_value("/pothole/com", res.center_of_mass)
            rt.data_management.set_value(
                "/pothole/surface_area", res.surface_area_m
            )
        else:
            rospy.logwarn(f"Response incorrect type.")
