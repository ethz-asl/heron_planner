#!/usr/bin/env python

from __future__ import annotations  # for type hinting
from typing import Callable

import os
import rospy
import rospkg
import numpy as np

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
            return self.generate_res(success=True, message=message, code=0)
        else:
            # command not recognised
            return self.generate_res(
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
            return self.generate_res(
                success=False, message="Empty command received", code=1
            )

        primary_command = command_parts[0].upper()

        # handle specific commands
        if primary_command == "MOVE":
            if len(command_parts) != 3:
                return self.generate_res(
                    success=False, message="Invalid MOVE command format", code=2
                )
            try:
                x = float(command_parts[1])
                y = float(command_parts[2])

                rospy.loginfo(f"Moving robot {x, y} [m] in (X, Y)")
                rospy.sleep(3.0)
                message = f"Robot successfully moved"
                return self.generate_res(success=True, message=message, code=0)
            except ValueError:
                return self.generate_res(
                    success=False,
                    message="Invalid MOVE command parameters",
                    code=3,
                )

        elif primary_command == "TURN":
            if len(command_parts) != 2:
                return self.generate_res(
                    success=False, message="Invalid TURN command format", code=2
                )
            try:
                angle = float(command_parts[1])
                rospy.loginfo(f"Turning robot by {angle} [rad]")
                message = f"Robot successfully turned"

                return self.generate_res(success=True, message=message, code=0)
            except ValueError:
                return self.generate_res(
                    success=False,
                    message="Invalid TURN command parameter",
                    code=3,
                )

        elif primary_command == "SET_DO":
            if len(command_parts) != 3:
                return self.generate_res(
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
                    return self.generate_res(
                        success=False,
                        message="Invalid on/off SET_DO format",
                        code=2,
                    )

                message = f"Setting digital output pin {pin} to {state_msg}"
                rospy.loginfo(message)
                rospy.sleep(2.0)
                return self.generate_res(success=True, message=message, code=0)
            except ValueError:
                return self.generate_res(
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

    def generate_res(
        self, success: bool, message: str, code: int
    ) -> SetCommandStringResponse:
        """utility function to create a standard response."""
        res = SetCommandStringResponse()
        res.ret.success = success
        res.ret.message = message
        res.ret.code = code
        return res

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    try:
        ugv_interface = UgvDummyInterface()
        ugv_interface.run()

    except rospy.ROSInterruptException as err:
        rospy.logwarn(f"Shutting down interface : {err}")
