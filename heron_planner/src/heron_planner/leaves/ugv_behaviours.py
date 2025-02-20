#!/usr/bin/env python3

import rospy
import ros_trees as rt

import heron_utils.transform_utils as utils

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose2D, Twist

from robot_simple_command_manager_msgs.msg import (
    CommandString,
    RobotSimpleCommandAction,
    RobotSimpleCommandGoal,
    RobotSimpleCommandResult,
)
from robot_simple_command_manager_msgs.srv import SetCommandStringRequest
from robotnik_navigation_msgs.msg import DockGoal
from heron_msgs.srv import FindOffsetRequest

################################################################################
############################# parameters from config ###############################
################################################################################

# TODO change CMD_MANAGER & CMD_SEQUENCER to action!!
CMD_MANAGER_ACTION = rospy.get_param(
    "ugv/cmd_manager_action", "/robot/command_manager/action"
)
CMD_SEQUENCER_ACTION = rospy.get_param(
    "ugv/cmd_sequencer_action", "/robot/command_sequencer/action"
)
CMD_MANAGER_SRV = rospy.get_param(
    "ugv/cmd_manager_srv", "/robot/command_manager/command"
)
CMD_SEQUENCER_SRV = rospy.get_param(
    "ugv/cmd_sequencer_srv", "/robot/command_sequencer/command"
)
MOVE_TO_ACTION = rospy.get_param("ugv/move_to_action", "/robot/arm/move_to")
PICKUP_FROM_ACTION = rospy.get_param(
    "ugv/pickup_from_action", "/robot/arm/pickup_from"
)
PLACE_ON_ACTION = rospy.get_param("ugv/place_on_action", "/robot/arm/place_on")
DOCK_ACTION = rospy.get_param("ugv/dock_action", "/robot/base/dock")
ODOM_TOPIC = rospy.get_param("ugv/odom_topic", "/robot/odom")
FIND_OFFSET_SRV = rospy.get_param("ugv/find_offset_srv", "/robot/find_offset")
GET_DEPOSIT_SRV = rospy.get_param(
    "ugv/get_deposit_srv", "/robot/get_deposit_sequence"
)

################################################################################
############################### leaf definitions ###############################
################################################################################


# class _CommandManager(rt.leaves_ros.ServiceLeaf):
#     def __init__(self, *args, **kwargs):
#         """Base class for sending commands to the command_manager"""
#         super(_CommandManager, self).__init__(
#             service_name=CMD_MANAGER_SRV, *args, **kwargs
#         )

# class _CommandSequencer(rt.leaves_ros.ServiceLeaf):
#     def __init__(self, *args, **kwargs):
#         """Base class for sending commands to the command_sequencer"""
#         super(_CommandSequencer, self).__init__(
#             service_name=CMD_SEQUENCER_SRV, *args, **kwargs
#         )

#TODO pick & place for cmd

class _CommandManager(rt.leaves_ros.ActionLeaf):
    def __init__(self, *args, **kwargs):
        """Base class for sending action goals to the command_manager"""
        super(_CommandManager, self).__init__(
            action_namespace=CMD_MANAGER_ACTION, *args, **kwargs
        )

class _CommandSequencer(rt.leaves_ros.ActionLeaf):
    def __init__(self, *args, **kwargs):
        """Base class for sending action goals to the command_sequencer"""
        super(_CommandSequencer, self).__init__(
            action_namespace=CMD_SEQUENCER_ACTION, *args, **kwargs
        )


class MoveArmTo(_CommandManager):
    CMD = "MOVE_ARM_TO"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(MoveArmTo, self).__init__(
            name=task_name if task_name else "Move arm to named position",
            load=True,
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> CommandString:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, str):
            rospy.loginfo(f"Moving arm to: {data}")
            cmd_str = MoveArmTo.CMD + " " + data
            rospy.logerr(f"Command string: {cmd_str}")
            
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError

    def _result_fn(self):
        res = self._default_result_fn()
        rospy.logerr(f"Result from MOVE_ARM_TO srv: {res}")
        return res


# TODO TAKE_SNAP man
class TakeSnap(_CommandManager):
    """this saves the current photo in the robot"""

    CMD = CommandString(command="TAKE_SNAP")

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(TakeSnap, self).__init__(
            name=task_name if task_name else "Take snap",
            load_value=TakeSnap.CMD,
            *args,
            **kwargs,
        )


class Move(_CommandManager):
    CMD = "MOVE"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(Move, self).__init__(
            name=task_name if task_name else "Move base in XY",
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
            cmd_str = f"{Move.CMD} {pose_arr[0]} {pose_arr[1]} {yaw:.2f}"
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )
        if isinstance(data, str):  
            rospy.logwarn(f"Sending command : {data}")
            return RobotSimpleCommandGoal(
                command=CommandString(command=data)
            )
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError


class GoToGPS(_CommandManager):
    CMD = "GOTO_GPS"

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
            cmd_str = f"{GoToGPS.CMD} {pose_arr[0]} {pose_arr[1]} {yaw:.2f}"
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError


class CustomCommandManager(_CommandManager):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(CustomCommandManager, self).__init__(
            name=task_name if task_name else "Command Manager",
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> CommandString:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, str):
            rospy.loginfo(f"Sending cmd manager req: {data}")
            return RobotSimpleCommandGoal(
                    command=CommandString(command=data)
                )
        else:
            rospy.logerr(f"Type {type(data)} is not str")
            raise ValueError


# TODO ROLLER_DOWN seq
class RollerSequence(_CommandSequencer):
    CMD = command="POT_"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(RollerSequence, self).__init__(
            name=task_name if task_name else "Roller Sequence",
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> CommandString:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, str):
            cmd_str = f"{RollerSequence.CMD}{data}"
            rospy.loginfo(f"Sending roller sequence: {cmd_str}")
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )
        else:
            rospy.logerr(f"Type {type(data)} is not str")
            raise ValueError



class RollerDown(_CommandSequencer):
    CMD = CommandString(command="ROLLER_DOWN")

    def __init__(self, *args, **kwargs) -> None:
        super(RollerDown, self).__init__(
            name="Lower Roller", load_value=RollerDown.CMD, *args, **kwargs
        )


class RollerUp(_CommandSequencer):
    CMD = CommandString(command="ROLLER_UP")

    def __init__(self, *args, **kwargs) -> None:
        super(RollerUp, self).__init__(
            name="Raise Roller", load_value=RollerUp.CMD, *args, **kwargs
        )


class RollerCommand(_CommandManager):
    CMD = "ROLLER_COMMAND"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(RollerCommand, self).__init__(
            name=task_name if task_name else "Command Roller",
            load=True,
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> CommandString:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, float):
            if data >= 0 and data <= 1:
                rospy.loginfo(f"Moving roller: {data}")
                cmd_str = f"{RollerCommand.CMD} {str(data)}"
                return RobotSimpleCommandGoal(
                        command=CommandString(command=cmd_str)
                    )
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


class BlowPothole(_CommandSequencer):
    """Moves robot forward 5cm 8x and blows"""

    CMD = CommandString(command="BLOW_POTHOLE")

    def __init__(self, *args, **kwargs) -> None:
        super(BlowPothole, self).__init__(
            name="BlowPothole", load_value=BlowPothole.CMD, *args, **kwargs
        )


class Deposit(_CommandSequencer):
    # TODO ask how does this work, do we know if open or not?
    CMD = "DEPOSIT"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(Deposit, self).__init__(
            name=task_name if task_name else "Deposit material",
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self):
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, int):
            rospy.loginfo(f"Activating deposit {data}")
            cmd_str = f"{Deposit.CMD} {str(data)}"
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )


# TODO use dummy deposit seq


class CustomCommandSequencer(_CommandSequencer):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(CustomCommandSequencer, self).__init__(
            name=task_name if task_name else "Command Sequncer",
            load_fn=self._load_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> CommandString:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, str):
            rospy.loginfo(f"Sending cmd sequencer req: {data}")
            return RobotSimpleCommandGoal(
                    command=CommandString(command=data)
                )
        else:
            rospy.logerr(f"Type {type(data)} is not str")
            raise ValueError


class MoveTo(rt.leaves_ros.ActionLeaf):
    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(MoveTo, self).__init__(
            name=task_name if task_name else "Move arm to named position",
            action_namespace=MOVE_TO_ACTION,
            *args,
            **kwargs,
        )


class PickUpFrom(_CommandManager):
    CMD = "PICK"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(PickUpFrom, self).__init__(
            name=task_name if task_name else "Pick up from named position",
            load=True,
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> CommandString:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, str):
            rospy.loginfo(f"Picking up from: {data}")
            cmd_str = PickUpFrom.CMD + " " + data
            
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError

    def _result_fn(self):
        res = self._default_result_fn()
        rospy.logerr(f"Result Pick: {res}")
        return res



class PlaceOn(_CommandManager):
    CMD = "PLACE"

    def __init__(self, task_name="", *args, **kwargs) -> None:
        super(PlaceOn, self).__init__(
            name=task_name if task_name else "Place on named position",
            load=True,
            load_fn=self._load_fn,
            result_fn=self._result_fn,
            *args,
            **kwargs,
        )

    def _load_fn(self) -> RobotSimpleCommandGoal:
        data = self._default_load_fn(auto_generate=False)

        if isinstance(data, str):
            rospy.loginfo(f"Placing on: {data}")
            cmd_str = PlaceOn.CMD + " " + data
            
            return RobotSimpleCommandGoal(
                    command=CommandString(command=cmd_str)
                )
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError

    def _result_fn(self):
        res = self._default_result_fn()
        rospy.logerr(f"Result Place: {res}")
        rospy.logerr(f"Type {type(res)}")
        return res

# TODO add pickup/place actions here!
# PICKUP -> robot/arm/pickup_from right_holder/left_holder (wip from raquel)
# TODO PICK manager
# TODO PLACE manager

# TODO DOCK name man -> similar to move_arm_to but to TF pos
# TODO OMNI_DOCK name man


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
            *args,
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
                maximum_velocity=self.max_vel,
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
