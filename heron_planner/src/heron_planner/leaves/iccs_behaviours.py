#!/usr/bin/env python3

import rospy
import ros_trees as rt

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped

################################################################################
############################# parameters from config ###############################
################################################################################

FIND_POTHOLE_SRV = rospy.get_param(
    "/iccs/find_pothole_srv", "/find_potholes"
)
FIND_CRACK_SRV = rospy.get_param(
    "/iccs/find_crack_srv", "/find_cracks"
)

################################################################################
############################### Leaf definitions ###############################
################################################################################


class FindPothole(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", save_bb_key=None, *args, **kwargs) -> None:
        super(FindPothole, self).__init__(
            name=task_name if task_name else "Find pothole",
            service_name=FIND_POTHOLE_SRV,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.save_bb_key = save_bb_key

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
            rospy.logwarn(f"pothole is {res.surface_area_m} m^2")
            return res
        else:
            rospy.logwarn(f"Response incorrect type.")
            return False

class FindCrack(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", save_bb_key=None, *args, **kwargs) -> None:
        super(FindCrack, self).__init__(
            name=task_name if task_name else "Find crack",
            service_name=FIND_CRACK_SRV,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.save_bb_key = save_bb_key

    def _result_fn(self):
        start_point = "/crack/start"
        end_point = "/crack/end"
        segmentation = "/crack/segmentation"
        success_key = "/crack/success"

        if self.save_bb_key is not None:
            bb_key = rt.data_management.get_value(self.save_bb_key)
            start_point = start_point + "/" + bb_key
            end_point = end_point + "/" + bb_key
            segmentation = segmentation + "/" + bb_key
            success_key = success_key + "/" + bb_key
 
        res = self._default_result_fn()
        rospy.logwarn(f"Found crack? : {res.success}")
        rt.data_management.set_value(success_key, res.success)

        if not res.success:
            return False

        if (
            isinstance(res.start_point, PointStamped) 
            and isinstance(res.end_point, PointStamped) 
            and isinstance(res.segmentation_mask, Image)
        ):

            rt.data_management.set_value(start_point, res.start_point)
            rt.data_management.set_value(end_point, res.end_point)
            rt.data_management.set_value(segmentation, res.segmentation_mask)
            return res
        else:
            rospy.logwarn(f"Response incorrect type.")
            return False

