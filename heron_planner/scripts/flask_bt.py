#!/usr/bin/env python

import visualizer as vis
import bt_rendering as render
import component_behaviours as comp_bt
import base_behaviours as base_bt
import arm_behaviours as arm_bt
import simple_flask_server
import threading
import requests
import functools

import rospy

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

        self.load_parameters()

        # self.tree_pub = rospy.Publisher("/bt_tree_svg", String, queue_size=10)
        # self.status_pub = rospy.Publisher("/bt_status", String, queue_size=10)
        self.hlp_status_pub = rospy.Publisher(
            "hlp/state", String, queue_size=10
        )

        self.root = py_trees.composites.Sequence(
            name="InspectionSequence", memory=True
        )

        at_gps = base_bt.AtGPS("At offset defect GPS?", self.defect_gps)
        go_to_gps = base_bt.GoToGPS("Go to offset defect", self.defect_gps)

        at_ee_pose = arm_bt.ArmAt("Arm at home?", "HOME")
        go_to_home = arm_bt.MoveTo("Move arm to home", "HOME")

        self.pothole_sel = py_trees.composites.Selector(
            "DefectSelector", children=[at_gps, go_to_gps], memory=True
        )

        self.home_sel = py_trees.composites.Selector(
            "HomeSelector", children=[at_ee_pose, go_to_home], memory=True
        )

        self.root.add_children([self.home_sel, self.pothole_sel])

        self.tree = py_trees.trees.BehaviourTree(self.root)

        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.publish_to_html(self.tree, snapshot_visitor)

        rospy.loginfo("starting flask server...")
        self.vis_thread = threading.Thread(
            target=simple_flask_server.start_flask_server, args=(self.tree,)
        )
        self.vis_thread.daemon = True
        self.vis_thread.start()

        # Service Servers for starting and stopping
        self.start_service = rospy.Service("start_bt", Trigger, self.start_bt)
        self.stop_service = rospy.Service("stop_bt", Trigger, self.stop_bt)
        self.pause_service = rospy.Service("pause_bt", Trigger, self.pause_bt)

    def load_parameters(self) -> None:
        self.defect_gps = NavSatFix(
            header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
            latitude=35,
            longitude=-125,
            altitude=0,
        )
        self.inspection_names = [
            "INSPECTION1",
            "INSPECTION2",
            "INSPECTION3",
            "INSPECTION4",
            "INSPECTION5",
        ]

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

    def publish_to_html(self, tree, snapshot_visitor):
        state = py_trees.display.ascii_tree(
            tree.root, snapshot_information=snapshot_visitor
        )
        self.update_status(str(state))

        graph = render.dot_graph(tree.root, include_status=True)
        svg_output = graph.create_svg()
        self.update_svg(svg_output)

    def update_status(self, state: str):
        url = "http://localhost:5000/update_status"
        try:
            response = requests.post(url, json={"state": state})
            response.raise_for_status()
            rospy.logwarn(f"Status successfully updated to server: {state}")
        except requests.exceptions.RequestException as err:
            rospy.logerr(f"Error updating status to server: {err}")

    def update_svg(self, svg):

        url = "http://localhost:5000/update_svg"
        try:
            response = requests.post(url, data=svg)
            response.raise_for_status()
            rospy.logwarn(
                f"svg successfully sent to server: {response.json()['path']}"
            )
        except requests.exceptions.RequestException as err:
            rospy.logerr(f"error sending svg to server: {err}")

    def update_state(self, tree):
        state = String()
        tip_behaviour = tree.tip()
        state.data = tip_behaviour.name

        self.hlp_status_pub.publish(state)

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

    def post_tick_handler(self, snapshot_visitor, tree):
        self.update_state(tree)
        self.publish_to_html(tree, snapshot_visitor)


def main():
    rospy.init_node("flask_test_bt")
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    node = FlaskTestBT()

    rospy.spin()


if __name__ == "__main__":
    main()
