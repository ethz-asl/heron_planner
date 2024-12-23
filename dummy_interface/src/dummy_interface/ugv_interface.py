#!/usr/bin/env python

from __future__ import annotations  # for type hinting
from typing import Callable

import os
import rospy
import rospkg
import numpy as np
import actionlib

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from robot_simple_command_manager_msgs.srv import (
    SetCommandString,
    SetCommandStringRequest,
    SetCommandStringResponse,
)
from heron_msgs.srv import (
    ChangeRobotMode,
    ChangeRobotModeRequest,
    ChangeRobotModeResponse,
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
        
        # start actions
        self.move_to_srv.start()
        self.move_to_pose_srv.start()
        self.pickup_from_srv.start()
        self.place_on_srv.start()

        rospy.loginfo("Dummy UGV interface services are ready.")

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

        else:
            # command not recognised
            return self.generate_res(
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
    
    def handle_move_to(self, goal: MoveToGoal):
        rospy.loginfo(f"MoveTo recieved: {goal.to}")
        feedback = MoveToFeedback(state="Moving to target location")
        self.move_to_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = MoveToResult(success=True, message=f"Moved to {goal.to}")
        feedback.state = "Completed"
        self.move_to_srv.set_succeed(res)
        rospy.loginfo(f"MoveTo completed {res}")

    def handle_move_to_pose(self, goal: MoveToPoseGoal):
        rospy.loginfo(f"MoveToPose recieved: {goal.pose}")
        feedback = MoveToPoseFeedback(state="Moving to target location")
        self.move_to_pose_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = MoveToPoseResult(success=True, message=f"Moved to {goal.pose}")
        feedback.state = "Completed"
        self.move_to_pose_srv.set_succeed(res)
        rospy.loginfo(f"MoveToPose completed {res}")

    def handle_pickup_from(self, goal: PickupFromGoal):

        rospy.loginfo(f"MoveTo recieved: {goal}")
        feedback = PickupFromFeedback(state="Moving to target location")
        self.pickup_from_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = PickupFromResult(success=True, message=f"Moved to {goal}")
        feedback.state = "Completed"
        self.pickup_from_srv.set_succeed(res)
        rospy.loginfo(f"PickUpFrom completed {res}")

    def handle_place_on(self, goal: PlaceOnGoal):
        rospy.loginfo(f"MoveTo recieved: {goal}")
        feedback = PlaceOnFeedback(state="Moving to target location")
        self.place_on_srv.publish_feedback(feedback)
        rospy.sleep(3.0)

        res = PlaceOnResult(success=True, message=f"Moved to {goal}")
        feedback.state = "Completed"
        self.place_on_srv.set_succeed(res)
        rospy.loginfo(f"PlaceOn completed {res}")


    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    try:
        ugv_interface = UgvDummyInterface()
        ugv_interface.run()

    except rospy.ROSInterruptException as err:
        rospy.logwarn(f"Shutting down interface : {err}")
