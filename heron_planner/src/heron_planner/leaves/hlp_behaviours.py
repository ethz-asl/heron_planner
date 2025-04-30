#!/usr/bin/env python3

import rospy
import py_trees as pt
import ros_trees as rt

import heron_utils.transform_utils as utils

from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from nav_msgs.srv import GetPlanRequest

from heron_msgs.srv import (
    SendImageToKafkaRequest,
    TransformPoseRequest,
    FindOffsetRequest,
)

################################################################################
############################### leaf definitions ###############################
################################################################################


class GetSynchedImages(rt.leaves_ros.ServiceLeaf):
    def __init__(
        self,
        task_name="",
        image_key="",
        save_bb_key=None,
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
        self.save_bb_key = save_bb_key

    def _result_fn(self):
        res = self._default_result_fn()

        if isinstance(res.image_rgb, Image) and self.save:
            if self.save_bb_key is not None:
                self.image_key = self.image_key + "/" + rt.data_management.get_value(self.save_bb_key)
                rospy.logwarn(f"Saving img to: {self.image_key}")

            if self.image_key is not None:
                rt.data_management.set_value(self.image_key, res.image_rgb)

        return res


class SendImageToKafka(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", msg="", *args, **kwargs):
        super(SendImageToKafka, self).__init__(
            name=task_name if task_name else "Send photo to Kafka",
            service_name="/kafka/publish_image",
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.msg = msg

    def _load_fn(self):
        img = self._default_load_fn(auto_generate=False)
        if isinstance(img, Image):
            req = SendImageToKafkaRequest(image=img, message=self.msg)
            return req
        else:
            rospy.logerr(f"Type {type(img)}: is incorrect")
            raise ValueError
        
    def _result_fn(self):
        res = self._default_result_fn()
        rospy.logerr(f"kafka response : {res}")
        return res

class FakeSendImageToKafka(rt.leaves.Leaf):

    def __init__(self, task_name="", msg="", *args, **kwargs):
        super(FakeSendImageToKafka, self).__init__(
            name=task_name if task_name else "Send photo to Kafka (fake)",
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )
        self.msg = msg

    def _load_fn(self):
        img = self._default_load_fn(auto_generate=False)
        if isinstance(img, Image):
            # here make req to send to kafka
            rospy.loginfo(f"Sending img to (fake) kafka: {self.msg}")
            rospy.loginfo(f"(fake) kafka responded: success")
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

class GenerateSimplePath(rt.leaves_ros.ServiceLeaf):
    def __init__(self, start, goal, task_name="", *args, **kwargs):
        super(GenerateSimplePath, self).__init__(
            name=task_name if task_name else "Generate simple path",
            load_fn=self._load_fn,            
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.start = start
        self.goal = goal

    def _load_fn(self):
        if isinstance(self.start, PoseStamped) and isinstance(self.goal, PoseStamped):
            req = GetPlanRequest(
                start=self.start,
                goal=self.goal
            )
            return req
        else:
            rospy.logerr(f"Start or Goal not pose stamped")
            raise ValueError
        
    def _result_fn(self):
        res = self._default_result_fn()

        if isinstance(res.plan, Path):
            rt.data_management.set_value("simple_path", res.plan)
            return True
        

class GenerateCrackPath(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", save_bb_key=None, *args, **kwargs):
        super(GenerateCrackPath, self).__init__(
            name=task_name if task_name else "Generate crack path",
            service_name="/hlp/generate_crack_path",
            result_fn=self._result_fn,
            *args,
            **kwargs,    
        )
        self.save_bb_key = save_bb_key

    def _result_fn(self):
        path = "/crack/path"
        middle_pose = "/crack/middle"
        if self.save_bb_key is not None:
            bb_key = rt.data_management.get_value(self.save_bb_key)
            path = path + "/" + bb_key
            middle_pose = middle_pose + "/" + bb_key

        res = self._default_result_fn()
        rospy.logwarn(f"Found path?: {res.success}")
        if not res.success:
            return False
    
        if isinstance(res.path, Path) and isinstance(res.middle_pose, PoseStamped) and self.save:
            rt.data_management.set_value(path, res.path)
            rt.data_management.set_value(middle_pose, res.middle_pose)

        return res

class FindOffset(rt.leaves_ros.ServiceLeaf):
    def __init__(
        self,
        defect,
        broadcast=True,
        broadcast_frame="offset",
        task_name="",
        *args,
        **kwargs,
    ):
        super(FindOffset, self).__init__(
            name=task_name if task_name else "Find offset pose",
            service_name="/hlp/find_offset",
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.defect = defect
        self.broadcast = broadcast
        self.broadcast_frame = broadcast_frame

    def _load_fn(self):
        pose = self._default_load_fn(auto_generate=False)
        if isinstance(pose, PoseStamped):
            req = FindOffsetRequest(
                defect_pose=pose,
                defect_type=self.defect,
                broadcast_to_tf=self.broadcast,
                broadcast_frame=self.broadcast_frame,
            )
            return req
        else:
            rospy.logerr(f"Type {type(pose)}: is incorrect")
            raise ValueError

    def _result_fn(self):
        res = self._default_result_fn()
        if res.success:
            pose_key = self.save_key if self.save_key else self.defect + "offset"
            rt.data_management.set_value(pose_key, res.offset_pose)
            return res.offset_pose
        rospy.logwarn(f"Error finding offset")
        return res.success

class GetPath(rt.leaves_ros.SubscriberLeaf):
    def __init__(
        self,
        task_name="",
        *args,
        **kwargs,
    ) -> None:
        super(GetPath, self).__init__(
            name=task_name if task_name else "Get path",
            topic_name="/hlp/path",
            topic_class=Path,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )


    def _result_fn(self) -> bool:
        path = self._default_result_fn()

        if isinstance(path, Path):
            rospy.loginfo(f"Getting path")
            return path
        else:
            rospy.logerr(f"Subscriber or key not valid types")
            raise ValueError

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
