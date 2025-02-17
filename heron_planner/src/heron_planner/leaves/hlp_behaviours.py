#!/usr/bin/env python3

import rospy
import py_trees as pt
import ros_trees as rt

import heron_utils.transform_utils as utils

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from heron_msgs.srv import (
    SendImageToKafkaRequest,
    TransformPoseRequest,
)

################################################################################
############################### leaf definitions ###############################
################################################################################


class GetSynchedImages(rt.leaves_ros.ServiceLeaf):
    def __init__(
        self,
        task_name="",
        image_key="img",
        *args,
        **kwargs,
    ) -> None:
        super(GetSynchedImages, self).__init__(
            name=task_name if task_name else "Get synched images",
            service_name="/hlp/get_synched_images",
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.image_key = image_key

    def _result_fn(self):
        res = self._default_result_fn()
        if isinstance(res.image_rgb, Image) and self.save:
            rt.data_management.set_value(self.image_key, res.image_rgb)
        return res


class SendImageToKafka(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", msg="", *args, **kwargs):
        super(SendImageToKafka, self).__init__(
            name=task_name if task_name else "Send photo to Kafka",
            service_name="/kafka/publish_image",
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )
        self.msg = msg

    def _load_fn(self):
        img = self._default_load_fn(auto_generate=False)
        if isinstance(img, Image):
            req = SendImageToKafkaRequest(image=img, message=self.msg)
            rospy.logerr(f"Send image to kafka req: {req}")
            return req
        else:
            rospy.logerr(f"Type {type(img)}: is incorrect")
            raise ValueError


class TransformPose(rt.leaves_ros.ServiceLeaf):
    def __init__(self, target_frame, task_name="", *args, **kwargs):
        super(TransformPose, self).__init__(
            name=task_name if task_name else "Transform pose",
            service_name="/hlp/transform_pose",
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.target_frame = target_frame

    def _load_fn(self):
        pose_in = self._default_load_fn(auto_generate=False)
        if isinstance(pose_in, PoseStamped):
            req = TransformPoseRequest(
                pose_in=pose_in,
                target_frame=self.target_frame,
            )
            return req
        else:
            rospy.logerr(f"Type {type(pose_in)}: is incorrect")
            raise ValueError

    def _result_fn(self):
        res = self._default_result_fn()
        if res.success:
            pose_key = self.save_key if self.save_key else "pose_out"
            rt.data_management.set_value(pose_key, res.pose_out)
            return res.pose_out
        rospy.logwarn(f"Transform failed.")
        return res.success


class PopFromList(rt.leaves.Leaf):

    def __init__(self, task_name="", pop_position=0, *args, **kwargs):
        super(PopFromList, self).__init__(
            name=task_name if task_name else "Pop from list",
            result_fn=self._pop_item,
            *args,
            **kwargs,
        )
        self.pop_position = pop_position

    def _pop_item(self):
        if not self.loaded_data:
            return None
        item = self.loaded_data.pop(self.pop_position)
        rospy.logerr(f"Current list item: {item}")
        if self.load_key is not None:
            rt.data_management.set_value(self.load_key, self.loaded_data)
        else:
            rt.data_management.set_last_value(self, self.loaded_data)
        return item
