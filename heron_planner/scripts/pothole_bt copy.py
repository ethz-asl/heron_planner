#!/usr/bin/env python

import heron_planner.behaviours.base_behaviours as base
import heron_planner.behaviours.arm_behaviours as arm
import functools

import rospy

import py_trees as pt

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class PotholeBT:
    def __init__(self):
        self.is_running = False
        self.is_paused = False
        self.previous_status = None
        self.target_inspection = ""

        self.load_parameters()
        self.load_subscribers()

        self.root = pt.composites.Sequence(
            name="InspectionSequence", memory=True
        )

        # arm home position
        at_home = arm.ArmAt("Arm at home?", "HOME")
        arm_to_home = arm.MoveTo("Move arm to home", "HOME")

        home_sel = pt.composites.Selector(
            "HomeSelector", children=[at_home, arm_to_home], memory=True
        )
        
        # base at start
        at_pothole_start = base.AtPose(
            "At pothole start?", self.pothole_start
        )
        go_to_pothole_start = base.GoToGPS(
            "Go to pothole start", self.pothole_start
        )
        pothole_sel = pt.composites.Selector(
            "PotholeOffsetSelector",
            children=[at_pothole_start, go_to_pothole_start],
            memory=True,
        )

        init_seq = pt.composites.Sequence(
            name="Before inspection",
            children=[home_sel, pothole_sel],
            memory=True,
        )

        self.root.add_children([init_seq])

        self.tree = pt.trees.BehaviourTree(self.root)

        snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)

        # Service Servers for starting and stopping
        self.start_service = rospy.Service("start_bt", Trigger, self.start_bt)
        self.stop_service = rospy.Service("stop_bt", Trigger, self.stop_bt)
        self.pause_service = rospy.Service("pause_bt", Trigger, self.pause_bt)

    def load_parameters(self) -> None:
        self.pothole_start = PoseStamped()

    def load_subscribers(self) -> None:
        self.pothole_start_sub = rospy.Subscriber(
            "/robot/pothole_start", PoseStamped, self.pothole_start_cb
        )

    def pothole_start_cb(self, msg: PoseStamped):
        self.pothole_start = msg

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

        if self.tree.root.status == pt.common.Status.SUCCESS:
            self.is_running = False
            rospy.loginfo("BT finished successfully")
            self.timer.shutdown()
            self.tree.interrupt()
        elif self.tree.root.status == pt.common.Status.FAILURE:
            self.is_running = False
            rospy.loginfo("BT terminated with failure")
            self.timer.shutdown()
            self.tree.interrupt()
        else:
            self.tree.tick()

    def post_tick_handler(self, snapshot_visitor, tree):
        self.update_state(tree)
        self.publish_to_html(tree, snapshot_visitor)


def main():
    rospy.init_node("pothole_inspection_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = PotholeBT()

    rospy.spin()

if __name__ == "__main__":
    main()
