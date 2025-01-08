#!/usr/bin/env python

import visualizer as vis
import bt_rendering as render
import component_behaviours as comp_bt
import base_behaviours as base_bt
import arm_behaviours as arm_bt
import flask_server
import threading

import rospy

import networkx as nx
import numpy as np
import py_trees

from geometry_msgs.msg import Pose
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class FlaskTestBT:
    def __init__(self):

        self.visualize_only = False
        self.is_running = False
        self.is_paused = False
        self.previous_status = None

        self.html_pub = rospy.Publisher("/bt_html_updates", String, queue_size=10)

        self.root = py_trees.composites.Sequence(
            name="TestSequence", memory=True
        )

        activate_blower = comp_bt.Blow("Blow pothole")
        lower_roller = comp_bt.LowerRoller("Lower roller")
        lift_roller = comp_bt.LiftRoller("Lift roller")

        move_forward_5cm = base_bt.Move("Move forward 5cm", "0.05", "0.0")

        wait_10s = base_bt.Wait("Wait 10s", "10")

        move_to = arm_bt.MoveTo("Move to home", "HOME")

        pothole_gps = NavSatFix()
        pothole_gps.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        pothole_gps.latitude = 35 # Random latitude
        pothole_gps.longitude = -125  # Random longitude
        pothole_gps.altitude = 0  # Altitude in meters
        at_gps = base_bt.AtGPS("At offset pothole GPS?", pothole_gps)
        go_to_gps = base_bt.GoToGPS("Go to offset pothole", pothole_gps)

        at_ee_pose = arm_bt.ArmAt("At HOME?", "HOME")
        go_to_home = arm_bt.MoveTo("Go to HOME", "HOME")

        self.roller = py_trees.composites.Sequence(
            name="RollerSequence", children=[lower_roller, wait_10s, lift_roller], memory=True
        )

        self.blow_seq = py_trees.composites.Sequence(
            name="BlowSeq", children=[activate_blower, wait_10s, move_forward_5cm], memory=True
        )

        self.pothole_sel = py_trees.composites.Selector(
            "PotholeSelector", children=[at_gps, go_to_gps], memory=True
        )

        self.home_sel = py_trees.composites.Selector(
            "HomeSelector", children=[at_ee_pose, go_to_home], memory=True 
        )

        self.root.add_children([self.home_sel, self.pothole_sel])

        self.tree = py_trees.trees.BehaviourTree(self.root)

        rospy.loginfo("starting flask server...")
        self.vis_thread = threading.Thread(target=flask_server.start_flask_server, args=(self.tree,))
        self.vis_thread.daemon = True
        self.vis_thread.start()
 
        # Service Servers for starting and stopping
        self.start_service = rospy.Service("start_bt", Trigger, self.start_bt)
        self.stop_service = rospy.Service("stop_bt", Trigger, self.stop_bt)
        self.pause_service = rospy.Service("pause_bt", Trigger, self.pause_bt)


    def start_bt(self, req) -> TriggerResponse:
        """"""
        if self.is_running:
            return TriggerResponse(
                success=False, message="BT is already running"
            )
        self.is_running = True
        rospy.loginfo("Starting the BT...")

        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick_bt)
        return TriggerResponse(success=True, message="BT started")

    def stop_bt(self, req) -> TriggerResponse:
        """"""
        if not self.is_running:
            return TriggerResponse(success=False, message="BT is not running")
        self.is_running = False
        rospy.loginfo("Stopping the BT...")
        # rospy.Time(rospy.Duration(0.1), self.tick_bt)
        self.timer.shutdown()
        return TriggerResponse(success=True, message="BT stopped")

    def pause_bt(self, req) -> TriggerResponse:
        if not self.is_running:
            return TriggerResponse(
                success=False,
                message="BT is not running, cannot pause/continue",
            )

        self.is_paused = not self.is_paused
        state = "paused" if self.is_paused else "continued"
        rospy.loginfo(f"BT {state}.")
        return TriggerResponse(success=True, message=f"BT {state}.")

    def tick_bt(self, event):
        """"""
        # rospy.logwarn(f"Ticking BT: is_running={self.is_running}, is_paused={self.is_paused}")
        if not self.is_running:
            return

        if self.is_paused:
            rospy.loginfo_once("BT is paused. Waiting to continue...")
            return

        if self.tree.root.status == py_trees.common.Status.SUCCESS:
            rospy.loginfo("BT finished successfully")
            self.is_running = False
            self.timer.shutdown()
        else:
            self.tree.tick()


def main():
    rospy.init_node("flask_test_bt")
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    node = FlaskTestBT()

    rospy.spin()


if __name__ == "__main__":
    main()
