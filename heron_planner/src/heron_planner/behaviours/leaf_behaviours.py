#!/usr/bin/env python3

import rospy
import py_trees as pt
import ros_trees as rt

import heron_utils.transform_utils as utils

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose2D, Twist

from robot_simple_command_manager_msgs.msg import CommandString
from robotnik_navigation_msgs.msg import DockGoal
from heron_msgs.srv import (
    SendImageToKafkaRequest,
    TransformPoseRequest,
    FindOffsetRequest,
)

################################################################################
############################### Leaf definitions ###############################
################################################################################


class _CommandManager(rt.leaves_ros.ServiceLeaf):
    def __init__(self, *args, **kwargs):
        """Base class for sending commands to the command_manager"""
        super(_CommandManager, self).__init__(
            service_name="/robot/command_manager/command", *args, **kwargs
        )


class _CommandSequencer(rt.leaves_ros.ServiceLeaf):
    def __init__(self, *args, **kwargs):
        """Base class for sending commands to the command_sequencer"""
        super(_CommandSequencer, self).__init__(
            service_name="/robot/command_sequencer/command", *args, **kwargs
        )


class MoveTo(rt.leaves_ros.ActionLeaf):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(MoveTo, self).__init__(
            name=task_name if task_name else "Move arm to named position",
            action_namespace="/robot/arm/move_to",
            *args,
            **kwargs,
        )


class ArmAt(rt.leaves_ros.SubscriberLeaf):
    def __init__(self, pose_name: str, task_name="", *args, **kwargs) -> None:
        super(ArmAt, self).__init__(
            name=task_name if task_name else "Arm at named position?",
            topic_name="/robot/arm/ee_pose_name",
            topic_class=String,
            result_fn=self._result_fn,
            # once_only=True,
            *args,
            **kwargs,
        )
        self.pose_name = pose_name

    def _result_fn(self):
        if self._default_result_fn() is not None:
            at_pose = self.pose_name == self._default_result_fn().data
            name = rt.data_management.get_value(self.pose_name)
            name = name if name else self.pose_name
            rospy.loginfo(f"Is arm at [{name}]? {at_pose}")
            return at_pose
        else:
            rospy.logwarn(f"subsriber msg not found")


class GoToGPS(_CommandManager):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(GoToGPS, self).__init__(
            name=task_name if task_name else "Move base to GPS",
            load=True,
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> str:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, PoseStamped):
            pose_arr = utils.array_from_pose(data.pose)
            yaw = utils.angle_from_quaternion(pose_arr[3:])
            return f"GOTO_GPS {pose_arr[0]} {pose_arr[1]} {yaw:.2f}"
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError


class Dock(rt.leaves_ros.ActionLeaf):
    def __init__(
        self,
        task_name="",
        robot_dock_frame="odom",
        dock_offset=None,
        max_lin=0.2,
        max_ang=0.1,
        *args,
        **kwargs,
    ) -> None:
        super(Dock, self).__init__(
            name=task_name if task_name else "Dock base to TF frame",
            action_namespace="/robot/base/dock",
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            * args,
            **kwargs,
        )
        self.robot_dock_frame = robot_dock_frame
        self.dock_offset = dock_offset if dock_offset else Pose2D(x=0, y=0)
        self.max_vel = Twist()
        self.max_vel.linear.x = max_lin
        self.max_vel.linear.y = max_lin
        self.max_vel.angular.z = max_ang

    def _load_fn(self):
        dock_frame = self._default_load_fn(auto_generate=False)
        if isinstance(dock_frame, str):
            req = DockGoal(
                dock_frame=dock_frame,
                robot_dock_frame=self.robot_dock_frame,
                dock_offset=self.dock_offset,
                maximum_velocity=self.max_vel
            )
            return req
        else:
            rospy.logerr(f"Type {type(dock_frame)}: is incorrect")
            raise ValueError
        
    def _result_fn(self):
        res = self._default_result_fn()
        rospy.loginfo(f"Dock action: {res.description}")
        return res.success

class AtPose(rt.leaves_ros.SubscriberLeaf):
    def __init__(
        self,
        pose_name: str = "",
        tol: float = 0.01,
        task_name="",
        *args,
        **kwargs,
    ) -> None:
        super(AtPose, self).__init__(
            name=task_name if task_name else "Base at Pose?",
            topic_name="/robot/odom",
            topic_class=Odometry,
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )
        self.pose_name = pose_name
        self.tol = tol

    def _load_fn(self):
        if self.pose_name:
            self.pose = rt.data_management.get_value(self.pose_name)
        else:
            self.pose = self._default_load_fn()

    def _result_fn(self) -> bool:
        odom = self._default_result_fn()
        pose = rt.data_management.get_value(self.pose_name)

        if isinstance(odom, Odometry) and isinstance(pose, PoseStamped):
            at_pose = utils.at_pose(odom.pose.pose, pose.pose, self.tol)
            rospy.loginfo(f"Is robot at pose [{self.pose_name}]? {at_pose}")
            return at_pose
        else:
            rospy.logerr(f"Subscriber or key not valid types")
            raise ValueError


class LiftRoller(_CommandSequencer):
    CMD = CommandString(command="LIFT_ROLLER")

    def __init__(self, *args, **kwargs) -> None:
        super(LiftRoller, self).__init__(
            name="Lift Roller", load_value=LiftRoller.CMD, *args, **kwargs
        )


class LowerRoller(_CommandSequencer):
    CMD = CommandString(command="LOWER_ROLLER")

    def __init__(self, *args, **kwargs) -> None:
        super(LowerRoller, self).__init__(
            name="Lower Roller", load_value=LowerRoller.CMD, *args, **kwargs
        )


class Blow(_CommandSequencer):
    CMD = CommandString(command="BLOW")

    def __init__(self, *args, **kwargs) -> None:
        super(Blow, self).__init__(
            name="Blow", load_value=Blow.CMD, *args, **kwargs
        )


class Deposit(_CommandSequencer):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(Deposit, self).__init__(
            name=task_name if task_name else "Deposit material", *args, **kwargs
        )


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


class FindPothole(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(FindPothole, self).__init__(
            name=task_name if task_name else "Find pothole",
            service_name="/iccs/find_pothole",
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
            rospy.logwarn(f"Pothole found: {res}")
            rt.data_management.set_value("/pothole/com", res.center_of_mass)
            rt.data_management.set_value(
                "/pothole/surface_area", res.surface_area_m
            )
            return res.success
        else:
            rospy.logwarn(f"Response incorrect type.")


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
            rospy.loginfo(f"sending {req}")
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
            service_name="/robot/find_offset",
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
            rospy.loginfo(f"sending {req}")
            return req
        else:
            rospy.logerr(f"Type {type(pose)}: is incorrect")
            raise ValueError

    def _result_fn(self):
        res = self._default_result_fn()
        rospy.logerr(f"offset res: {res}")
        if res.success:
            pose_key = self.save_key if self.save_key else "offset_pose"
            rt.data_management.set_value(pose_key, res.offset_pose)
            return res.offset_pose
        rospy.logwarn(f"Error finding offset")
        return res.success


class GetDepositSeq(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", *args, **kwargs):
        super(GetDepositSeq, self).__init__(
            name=task_name if task_name else "Find offset pose",
            service_name="/robot/get_deposit_sequence",
            *args,
            **kwargs,
        )


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


class Print(rt.leaves.Leaf):

    def __init__(self, *args, **kwargs):
        super(Print, self).__init__(
            "Print", result_fn=self._print, *args, **kwargs
        )

    def _print(self):
        print(self.loaded_data)
        return True


class PrintObjects(rt.leaves.Leaf):

    def __init__(self, *args, **kwargs):
        super(PrintObjects, self).__init__(
            name="Print Objects", result_fn=self._print_objects, *args, **kwargs
        )

    def _print_objects(self):
        if self.loaded_data is None or not self.loaded_data:
            print("The detector found no objects!")
        else:
            print(
                "The detector found %d objects at the following coordinates:"
                % len(self.loaded_data)
            )
            for o in self.loaded_data:
                print(
                    "\t'%s' of pixel dimensions %dx%d @ top left coordinates:"
                    " (%d,%d)"
                    % (o.class_label, o.width, o.height, o.x_left, o.y_top)
                )

        return True


class WaitForEnterKey(rt.leaves.Leaf):

    def __init__(self, *args, **kwargs):
        super(WaitForEnterKey, self).__init__(
            name="Wait for Enter Key",
            result_fn=self._wait_for_enter,
            *args,
            **kwargs,
        )

    def _wait_for_enter(self):
        # NOTE: this is blocking within a leaf ... typically BAD
        input(
            self.loaded_data
            if self.loaded_data
            else "Press enter to continue: "
        )
        return True


class SaveData(rt.leaves.Leaf):
    def __init__(self, data, task_name="", *args, **kwargs):
        super(SaveData, self).__init__(
            name=task_name if task_name else "Data generator",
            load_value=data,
            save=True,
            *args,
            **kwargs,
        )


################################################################################
############################## Branch definitions ##############################
################################################################################


class ArmToHome(pt.composites.Selector):
    CMD = "HOME"

    def __init__(self, *args, **kwargs) -> None:
        super(ArmToHome, self).__init__(
            name="Send arm to home",
            children=[
                ArmAt(task_name="Arm at home?", pose_name="HOME"),
                MoveTo(task_name="Move arm to home", load_value="HOME"),
            ],
        )


class BaseToStart(pt.composites.Selector):
    def __init__(self, *args, **kwargs) -> None:
        super(BaseToStart, self).__init__(
            name="BaseToStartSel",
            children=[AtPose(pose_name="start"), GoToGPS(load_key="start")],
        )


################################################################################
######################### Tree definition & Execution ##########################
################################################################################


def object_list_from_response(leaf, response):
    return leaf._default_save_fn(response.objects)


if __name__ == "__main__":
    rospy.init_node("test_ros_trees")

    # Example Pose for GoToGPS
    target_pose = PoseStamped()
    target_pose.pose.position.x = 10.0
    target_pose.pose.position.y = 5.0
    target_pose.pose.orientation.z = 0.707
    target_pose.pose.orientation.w = 0.707

    # Create a simple behavior tree
    # root = pt.composites.Sequence(name="Root Sequence")

    save_start = SaveData(target_pose, save_key="start")

    # # Add the GoToGPS leaf to the tree
    # root.add_child(save_start)

    # # Display the tree structure

    root = rt.trees.BehaviourTree(
        "Test ros_trees",
        pt.composites.Sequence(
            name="Test seq",
            children=[
                WaitForEnterKey(),
                pt.composites.Sequence(
                    name="Started?",
                    children=[save_start, ArmToHome(), BaseToStart()],
                ),
            ],
        ),
    )
    root.visualise()
    root.run(hz=30, push_to_start=True, log_level="WARN")
