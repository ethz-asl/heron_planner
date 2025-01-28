#!/usr/bin/env python3

import rospy
import py_trees as pt
import ros_trees as rt

import heron_utils.transform_utils as utils

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from heron_msgs.msg import MoveToActionGoal
from robot_simple_command_manager_msgs.msg import CommandString
from robot_simple_command_manager_msgs.srv import (
    SetCommandString,
    SetCommandStringRequest,
    SetCommandStringResponse,
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


# utility to format CommandString
class CommandFormatter(rt.leaves.Leaf):
    _key_mapping = {}

    @staticmethod
    def format_goto_gps(pose: PoseStamped) -> str:
        pose_arr = utils.array_from_pose(pose.pose)
        yaw = utils.angle_from_quaternion(pose_arr[3:])
        return f"GOTO_GPS {pose_arr[0]} {pose_arr[1]} {yaw}"

    @staticmethod
    def set_key(key: str, command: str) -> None:
        CommandFormatter._key_mapping[key] = command

    @staticmethod
    def get_key(key: str) -> str:
        return CommandFormatter._key_mapping.get(key, None)


class SetKey(rt.leaves.Leaf):
    """want to set initial keys here"""

    def __init__(self, pop_position=0, *args, **kwargs):
        super(PopFromList, self).__init__(
            "Pop from list", result_fn=self._pop_item, *args, **kwargs
        )
        self.pop_position = pop_position

    def _pop_item(self):
        if not self.loaded_data:
            return None
        item = self.loaded_data.pop(self.pop_position)
        if self.load_key is not None:
            rt.data_management.set_value(self.load_key, self.loaded_data)
        else:
            rt.data_management.set_last_value(self, self.loaded_data)
        return item


class MoveTo(rt.leaves_ros.ActionLeaf):
    def __init__(self, *args, **kwargs) -> None:
        super(MoveTo, self).__init__(
            name="Move arm to named position",
            action_namespace="/robot/arm/move_to",
            *args,
            **kwargs,
        )


class ArmAt(rt.leaves_ros.SubscriberLeaf):
    def __init__(self, *args, **kwargs) -> None:
        super(ArmAt, self).__init__(
            name="Arm at named position?",
            topic_name="/robot/arm/ee_pose_name",
            topic_class=String,
            debug=rt.debugging.DebugMode.INSTANT_FAILURE,
            *args,
            **kwargs,
        )


class GoToGPS(_CommandManager):
    def __init__(self, *args, **kwargs) -> None:
        super(GoToGPS, self).__init__(
            name="Move base to GPS",
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
            return f"GOTO_GPS {pose_arr[0]} {pose_arr[1]} {yaw}"
        else:
            rospy.logerr(f"Type {type(data)}: is incorrect")
            raise ValueError


class AtPose(rt.leaves_ros.SubscriberLeaf):
    def __init__(self, key: str, tol: float = 0.01, *args, **kwargs) -> None:
        super(AtPose, self).__init__(
            name="Base at Pose?",
            topic_name="/robot/odom",
            topic_class=Odometry,
            result_fn=self._result_fn,
            debug=rt.debugging.DebugMode.INSTANT_FAILURE,
            *args,
            **kwargs,
        )
        self.key = key
        self.tol = tol

    def _result_fn(self) -> bool:
        odom = self._default_result_fn()
        pose = rt.data_management.get_key(self.key)

        if isinstance(odom, Odometry) and isinstance(pose, PoseStamped):
            at_pose = utils.at_pose(odom.pose, pose.pose, self.tol)
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


class PopFromList(rt.leaves.Leaf):

    def __init__(self, pop_position=0, *args, **kwargs):
        super(PopFromList, self).__init__(
            "Pop from list", result_fn=self._pop_item, *args, **kwargs
        )
        self.pop_position = pop_position

    def _pop_item(self):
        if not self.loaded_data:
            return None
        item = self.loaded_data.pop(self.pop_position)
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
    def __init__(self, data, *args, **kwargs):
        super(SaveData, self).__init__(
            "Data generator",
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
            name="HomeSel",
            children=[ArmAt(load_value="HOME"), MoveTo(load_value="HOME")],
        )


class BaseToStart(pt.composites.Selector):
    def __init__(self, *args, **kwargs) -> None:
        super(BaseToStart, self).__init__(
            name="BaseToStartSel",
            children=[AtPose(key="start"), GoToGPS(load_key="start_gps")],
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
    # pt.display.render_dot_tree(root)

    rt.trees.BehaviourTree(
        "Test ros_trees",
        pt.composites.Sequence(
            name="Test seq",
            children=[
                WaitForEnterKey(load_value="Press enter to start ... "),
                pt.composites.Sequence(
                    name="Started?",
                    children=[save_start, ArmToHome(), BaseToStart()],
                ),
            ],
        ),
    ).run()
