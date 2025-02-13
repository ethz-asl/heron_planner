#!/usr/bin/env python3

import rospy
import ros_trees as rt

import heron_utils.transform_utils as utils

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose2D, Twist

from robot_simple_command_manager_msgs.msg import CommandString
from robotnik_navigation_msgs.msg import DockGoal
from heron_msgs.srv import FindOffsetRequest

################################################################################
############################# parameters from config ###############################
################################################################################

CMD_MANAGER_SRV = rospy.get_param("ugv/cmd_manager_srv", "/robot/command_manager/command")
CMD_SEQUENCER_SRV = rospy.get_param("ugv/cmd_sequencer_srv", "/robot/command_sequencer/command")
MOVE_TO_ACTION = rospy.get_param("ugv/move_to_action", "/robot/arm/move_to")
DOCK_ACTION = rospy.get_param("ugv/dock_action", "/robot/base/dock")
ODOM_TOPIC = rospy.get_param("ugv/odom_topic", "/robot/odom")
FIND_OFFSET_SRV = rospy.get_param("ugv/find_offset_srv", "/robot/find_offset")
GET_DEPOSIT_SRV = rospy.get_param("ugv/get_deposit_srv", "/robot/get_deposit_sequence")

################################################################################
############################### leaf definitions ###############################
################################################################################


class _CommandManager(rt.leaves_ros.ServiceLeaf):
    def __init__(self, *args, **kwargs):
        """Base class for sending commands to the command_manager"""
        super(_CommandManager, self).__init__(
            service_name=CMD_MANAGER_SRV, *args, **kwargs
        )


class _CommandSequencer(rt.leaves_ros.ServiceLeaf):
    def __init__(self, *args, **kwargs):
        """Base class for sending commands to the command_sequencer"""
        super(_CommandSequencer, self).__init__(
            service_name=CMD_SEQUENCER_SRV, *args, **kwargs
        )


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


class MoveTo(rt.leaves_ros.ActionLeaf):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(MoveTo, self).__init__(
            name=task_name if task_name else "Move arm to named position",
            action_namespace=MOVE_TO_ACTION,
            *args,
            **kwargs,
        )

#TODO add pickup/place/movetopose actions here!

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
            action_namespace=DOCK_ACTION,
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
            topic_name=ODOM_TOPIC,
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
            self.pose = self._default_load_fn(auto_generate=False)

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
            service_name=FIND_OFFSET_SRV,
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
            pose_key = self.save_key if self.save_key else "offset_pose"
            rt.data_management.set_value(pose_key, res.offset_pose)
            return res.offset_pose
        rospy.logwarn(f"Error finding offset")
        return res.success


class GetDepositSeq(rt.leaves_ros.ServiceLeaf):
    def __init__(self, task_name="", *args, **kwargs):
        super(GetDepositSeq, self).__init__(
            name=task_name if task_name else "Find offset pose",
            service_name=GET_DEPOSIT_SRV,
            *args,
            **kwargs,
        )

