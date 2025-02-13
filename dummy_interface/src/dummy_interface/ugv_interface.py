#!/usr/bin/env python

from __future__ import annotations  # for type hinting

import rospy
import actionlib
import tf2_ros

from geometry_msgs.msg import PoseStamped

import heron_utils.transform_utils as utils

from robot_simple_command_manager_msgs.srv import (
    SetCommandString,
    SetCommandStringRequest,
    SetCommandStringResponse,
)
from robotnik_navigation_msgs.msg import DockAction, DockGoal, DockResult
from heron_msgs.srv import (
    ChangeRobotMode,
    ChangeRobotModeRequest,
    ChangeRobotModeResponse,
    GetDepositSeq,
    GetDepositSeqRequest,
    GetDepositSeqResponse,
    FindOffset,
    FindOffsetRequest,
    FindOffsetResponse,
)
from heron_msgs.msg import (
    MoveToAction,
    MoveToGoal,
    MoveToFeedback,
    MoveToResult,
    MoveToPoseAction,
    MoveToPoseGoal,
    MoveToPoseFeedback,
    MoveToPoseResult,
    PickupFromAction,
    PickupFromGoal,
    PickupFromFeedback,
    PickupFromResult,
    PlaceOnAction,
    PlaceOnGoal,
    PlaceOnFeedback,
    PlaceOnResult,
)
from robot_simple_command_manager_msgs.msg import ReturnMessage

"""
Add here all the items for making a interface to ROB

E.G.
Sequences
Commands
as SRV requests!
publish status to kafka...

"""


class UgvDummyInterface:
    """ """

    def __init__(self) -> None:
        rospy.init_node("dummy_ugv_interface")

        self.load_parameters()

        # services
        self.cmd_sequencer_srv = rospy.Service(
            "/robot/command_sequencer/command",
            SetCommandString,
            self.handle_cmd_sequencer,
        )

        self.cmd_manager_srv = rospy.Service(
            "/robot/command_manager/command",
            SetCommandString,
            self.handle_cmd_manager,
        )

        self.cmd_manager_srv = rospy.Service(
            "/robot/change_mode",
            ChangeRobotMode,
            self.handle_change_mode,
        )
        self.get_deposit_srv = rospy.Service(
            "/robot/get_deposit_sequence",
            GetDepositSeq,
            self.handle_deposit_seq,
        )
        self.find_offset_srv = rospy.Service(
            "/robot/find_offset",
            FindOffset,
            self.handle_find_offset,
        )

        # actions
        self.move_to_srv = actionlib.SimpleActionServer(
            "/robot/arm/move_to",
            MoveToAction,
            execute_cb=self.handle_move_to,
            auto_start=False,
        )
        self.move_to_pose_srv = actionlib.SimpleActionServer(
            "/robot/arm/move_to_pose",
            MoveToPoseAction,
            execute_cb=self.handle_move_to_pose,
            auto_start=False,
        )
        self.pickup_from_srv = actionlib.SimpleActionServer(
            "/robot/arm/pickup_from",
            PickupFromAction,
            execute_cb=self.handle_pickup_from,
            auto_start=False,
        )
        self.place_on_srv = actionlib.SimpleActionServer(
            "/robot/arm/place_on",
            PlaceOnAction,
            execute_cb=self.handle_place_on,
            auto_start=False,
        )
        self.dock_srv = actionlib.SimpleActionServer(
            "/robot/base/dock",
            DockAction,
            execute_cb=self.handle_dock,
            auto_start=False,
        )

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))  # Store 5 seconds of TF history
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.cur_offset = None # offset transform to be broadcast

        # start actions
        self.move_to_srv.start()
        self.move_to_pose_srv.start()
        self.pickup_from_srv.start()
        self.place_on_srv.start()
        self.dock_srv.start()

        rospy.loginfo("Dummy UGV interface services are ready.")
        self.broadcast_timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)

    def load_parameters(self) -> None:
        self.map_frame = rospy.get_param("map_frame", "robot_map")


    def handle_cmd_sequencer(
        self, req: SetCommandStringRequest
    ) -> SetCommandStringResponse:
        """"""
        rospy.loginfo(f"Recieved cmd : {req.command}")

        sequences = (
            "LOWER_ROLLER",
            "LOWER_ROLLER_HALF",
            "LIFT_ROLLER",
            "BLOW",
            "DEPOSIT_1",
            "DEPOSIT_2",
            "DEPOSIT_3",
            "ENABLE_PAINT_MS",
            "DISABLE_PAINT_MS",
        )

        if req.command in sequences:
            rospy.loginfo(f"Sending sequence {req.command} to robot.")
            rospy.sleep(4.0)
            message = f"Sequence {req.command} finished."
            return self.generate_cmd_res(success=True, message=message, code=0)
        else:
            # command not recognised
            return self.generate_cmd_res(
                success=False,
                message=f"Unknown command: {req.command}",
                code=4,
            )

    def handle_cmd_manager(
        self, req: SetCommandStringRequest
    ) -> SetCommandStringResponse:
        """handling inputs from cmd manager"""
        rospy.loginfo(f"Recieved cmd : {req.command}")

        # parse the command
        command_parts = req.command.split()
        if not command_parts:
            return self.generate_cmd_res(
                success=False, message="Empty command received", code=1
            )

        primary_command = command_parts[0].upper()

        # handle specific commands
        if primary_command == "MOVE":
            if len(command_parts) != 3:
                return self.generate_cmd_res(
                    success=False, message="Invalid MOVE command format", code=2
                )
            try:
                x = float(command_parts[1])
                y = float(command_parts[2])

                rospy.loginfo(f"Moving robot {x, y} [m] in (X, Y)")
                rospy.sleep(3.0)
                message = f"Robot successfully moved"
                return self.generate_cmd_res(
                    success=True, message=message, code=0
                )
            except ValueError:
                return self.generate_cmd_res(
                    success=False,
                    message="Invalid MOVE command parameters",
                    code=3,
                )

        elif primary_command == "TURN":
            if len(command_parts) != 2:
                return self.generate_cmd_res(
                    success=False, message="Invalid TURN command format", code=2
                )
            try:
                angle = float(command_parts[1])
                rospy.loginfo(f"Turning robot by {angle} [rad]")
                message = f"Robot successfully turned"

                return self.generate_cmd_res(
                    success=True, message=message, code=0
                )
            except ValueError:
                return self.generate_cmd_res(
                    success=False,
                    message="Invalid TURN command parameter",
                    code=3,
                )

        elif primary_command == "SET_DO":
            if len(command_parts) != 3:
                return self.generate_cmd_res(
                    success=False,
                    message="Invalid SET_DO command format",
                    code=2,
                )
            try:
                pin = int(command_parts[1])
                on_state = command_parts[2].lower() in ("true", "1", "on")
                off_state = command_parts[2].lower() in ("false", "0", "off")
                if on_state:
                    state_msg = "ON"
                elif off_state:
                    state_msg = "OFF"
                else:
                    return self.generate_cmd_res(
                        success=False,
                        message="Invalid on/off SET_DO format",
                        code=2,
                    )

                message = f"Setting digital output pin {pin} to {state_msg}"
                rospy.loginfo(message)
                rospy.sleep(2.0)
                return self.generate_cmd_res(
                    success=True, message=message, code=0
                )
            except ValueError:
                return self.generate_cmd_res(
                    success=False,
                    message="Invalid SET_DO command parameters",
                    code=3,
                )

        elif primary_command == "WAIT":
            if len(command_parts) != 2:
                return self.generate_cmd_res(
                    success=False, message="Invalid WAIT command format", code=2
                )
            try:
                wait_time = float(command_parts[1])
                rospy.loginfo(f"Waiting for {wait_time} [s]")
                message = f"Waiting for {wait_time} [s]"
                rospy.sleep(wait_time)
                return self.generate_cmd_res(
                    success=True, message=message, code=0
                )
            except ValueError:
                return self.generate_cmd_res(
                    success=False,
                    message="Invalid WAIT command parameter",
                    code=3,
                )
        elif primary_command == "GOTO_GPS":
            if len(command_parts) != 4:
                return self.generate_cmd_res(
                    success=False, message="Invalid MOVE command format", code=2
                )
            try:
                x = float(command_parts[1])
                y = float(command_parts[2])
                yaw = float(command_parts[3])

                rospy.loginfo(
                    f"Moving robot {x, y} [m] in (X, Y), in {yaw} [rad] direction"
                )
                rospy.sleep(3.0)
                message = f"Robot successfully moved"
                return self.generate_cmd_res(
                    success=True, message=message, code=0
                )
            except ValueError:
                return self.generate_cmd_res(
                    success=False,
                    message="Invalid MOVE command parameters",
                    code=3,
                )

        else:
            # command not recognised
            return self.generate_cmd_res(
                success=False,
                message=f"Unknown command: {primary_command}",
                code=4,
            )

    def handle_change_mode(
        self, req: ChangeRobotModeRequest
    ) -> ChangeRobotModeResponse:
        """do something here like checking for URDF"""

        mode_parts = req.mode.split()
        if not mode_parts:
            return self.generate_mode_res(
                success=False, message="Empty command received", code=1
            )

        primary_mode = mode_parts[0].upper()

        modes = ("URDF", "MOVEIT", "UR")

        if primary_mode == "URDF":
            rospy.loginfo(f"Changing URDF model to {mode_parts[1]}")
            rospy.sleep(3.0)
            message = f"URDF successfully changed"
            return self.generate_mode_res(success=True, message=message)

        elif primary_mode == "MOVEIT":
            rospy.loginfo(f"Changing MoveIt planner to {mode_parts[1]}")
            rospy.sleep(3.0)
            message = f"MoveIt planner successfully changed"
            return self.generate_mode_res(success=True, message=message)

        elif primary_mode == "UR":
            rospy.loginfo(f"Changing UR end-effector to {mode_parts[1]}")
            rospy.sleep(3.0)
            message = f"UR end-effector successfully changed"
            return self.generate_mode_res(success=True, message=message)

        else:
            # command not recognised
            return self.generate_mode_res(
                success=False,
                message=f"Unknown mode: {req.mode}",
                code=4,
            )

    def generate_mode_res(
        self, success: bool, message: str
    ) -> ChangeRobotModeResponse:
        """utility function for standard mode reponse"""
        res = ChangeRobotModeResponse()
        res.success = success
        res.msg = message
        return res

    def generate_cmd_res(
        self, success: bool, message: str, code: int
    ) -> SetCommandStringResponse:
        """utility function to create a standard cmd string response."""
        res = SetCommandStringResponse()
        res.ret.success = success
        res.ret.message = message
        res.ret.code = code
        return res

    def handle_deposit_seq(
        self, req: GetDepositSeqRequest
    ) -> GetDepositSeqResponse:
        # dummy deposit sizes
        min_pothole = 0.08
        max_pothole = 0.2
        small_pothole = 0.12
        large_pothole = 0.15

        res = GetDepositSeqResponse()

        if (  # pothole too big or too small
            req.surface_area_m < min_pothole or req.surface_area_m > max_pothole
        ):
            rospy.logwarn(
                f"Pothole must be between [{min_pothole, max_pothole}] [m^2]"
            )
            rospy.logwarn(
                f"Detected surface area is [{req.surface_area_m}] [m^2]"
            )
            res.deposit_sequence = "ERROR"
        elif (  # pothole small size
            req.surface_area_m >= min_pothole
            and req.surface_area_m < small_pothole
        ):
            res.deposit_sequence = "DEPOSIT_1"
        elif (  # pothole medium size
            req.surface_area_m >= small_pothole
            and req.surface_area_m < large_pothole
        ):
            res.deposit_sequence = "DEPOSIT_2"
        elif (  # pothole large size
            req.surface_area_m >= large_pothole
            and req.surface_area_m <= max_pothole
        ):
            res.deposit_sequence = "DEPOSIT_3"

        return res

    def handle_find_offset(self, req: FindOffsetRequest) -> FindOffsetResponse:

        res = FindOffsetResponse()
        res.success = True

        defect_offsets = {
            "POTHOLE": 1.0,
            "CRACKS": 0.7,
            "ROAD_MARKINGS": 0.5,
            "CONE_PLACE": 0.7,
            "CONE_PICKUP": 0.7,
        }

        if req.defect_type in defect_offsets:
            res.offset_pose = utils.find_offset_from_pose(
                req.defect_pose, offset_dist=defect_offsets[req.defect_type], towards=False,
            )
        else:
            rospy.logwarn(f"Invalid defect type: {req.defect_type}")
            res.offset_pose = None
            res.success = False


        if req.broadcast_to_tf and res.success and res.offset_pose:
            rospy.loginfo(f"Broadcasting to TF on frame: {req.broadcast_frame}")
            self.cur_offset = utils.transform_from_pose_stamped(
                res.offset_pose, req.broadcast_frame
            )

        return res

    def handle_move_to(self, goal: MoveToGoal):
        rospy.loginfo(f"MoveTo recieved: {goal.to}")
        feedback = MoveToFeedback(state=f"Moving to target location")
        self.move_to_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = MoveToResult(success=True, message=f"Moved to {goal.to}")
        feedback.state = "Completed"
        self.move_to_srv.set_succeeded(res)
        rospy.loginfo(f"MoveTo completed {res}")

    def handle_move_to_pose(self, goal: MoveToPoseGoal):
        rospy.loginfo(f"MoveToPose recieved: {goal.pose}")
        feedback = MoveToPoseFeedback(state="Moving to target location")
        self.move_to_pose_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = MoveToPoseResult(success=True, message=f"Moved to {goal.pose}")
        feedback.state = "Completed"
        self.move_to_pose_srv.set_succeeded(res)
        rospy.loginfo(f"MoveToPose completed {res}")

    def handle_pickup_from(self, goal: PickupFromGoal):

        rospy.loginfo(f"MoveTo recieved: {goal.location}")
        feedback = PickupFromFeedback(state="Moving to target location")
        self.pickup_from_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = PickupFromResult(
            success=True, message=f"Moved to {goal.location}"
        )
        feedback.state = "Completed"
        self.pickup_from_srv.set_succeeded(res)
        rospy.loginfo(f"PickUpFrom completed {res}")

    def handle_place_on(self, goal: PlaceOnGoal):
        rospy.loginfo(f"MoveTo recieved: {goal.location}")
        feedback = PlaceOnFeedback(state="Moving to target location")
        self.place_on_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = PlaceOnResult(success=True, message=f"Moved to {goal.location}")
        feedback.state = "Completed"
        self.place_on_srv.set_succeeded(res)
        rospy.loginfo(f"PlaceOn completed {res}")

    def handle_dock(self, goal: DockGoal):
        rospy.loginfo(f"Dock recieved goal: {goal.dock_frame}")

        res = DockResult()
        rospy.sleep(4)
        if not self.frames_exist_tf(goal.dock_frame, goal.robot_dock_frame):
            res.success = False
            res.description = f"Docking failed, frames not in TF"
            self.dock_srv.set_aborted(res)
            return

        res.success = True
        res.description = f"Dock to {goal.dock_frame} completed successfully."
        self.dock_srv.set_succeeded(res)

    def frames_exist_tf(self, frame1, frame2):
        try:
            if self.tf_buffer.can_transform(
                frame1, frame2, rospy.Time(0), rospy.Duration(1.0)
            ): 
                return True
            else:
                rospy.logwarn(
                    f"Link between {frame1} and {frame2} not found in TF tree"
                )
                return False
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as err:
            rospy.logerr(f"TF Error: {err}")
            return False
        
    def broadcast_tf(self, event):
        if self.cur_offset:
            rospy.logwarn_once(f"broadcasting to TF!")
            self.cur_offset.header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(self.cur_offset)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    try:
        ugv_interface = UgvDummyInterface()
        ugv_interface.run()

    except rospy.ROSInterruptException as err:
        rospy.logwarn(f"Shutting down interface : {err}")
