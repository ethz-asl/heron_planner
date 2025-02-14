#!/usr/bin/env python

import rospy
import threading
import functools

import py_trees as pt
import ros_trees as rt

import heron_utils.bt_runner as bt_runner
import heron_utils.bt_rendering as render

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class BaseBT:
    def __init__(self, name: str) -> None:
        self.visualize_only = False
        self.is_running = False
        self.is_paused = False
        self.previous_status = None

        self.bb = pt.Blackboard()
        self.load_parameters()
        self.save_to_blackboard()

        self.root = self.build_root()
        self.tree = bt_runner.BehaviourTreeRunner(name, self.root)
        self.start_svg_publisher()

        self.hlp_status_pub = rospy.Publisher(
            "hlp/state", String, queue_size=10
        )
        self.hlp_tree_pub = rospy.Publisher(
            "hlp/html", String, queue_size=1
        )

        self.start_servers()

    def load_parameters(self) -> None:
        raise NotImplementedError("Subclasses must implement load_parameters().")

    def save_to_blackboard(self) -> None:
        raise NotImplementedError("Subclasses must implement save_to_blackboard().")
  
    def build_root(self) -> pt.behaviour.Behaviour:
        raise NotImplementedError("Subclasses must implement build_root().")
   
    def publish_svg_tree(self) -> None:
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            if self.tree and self.tree.root:
                graph = render.dot_graph(self.tree.root, include_status=True)
                svg_tree = graph.create_svg().decode("utf-8")
                self.hlp_tree_pub.publish(svg_tree)
            rate.sleep()

    def start_svg_publisher(self) -> None:
        # to update svg
        snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)

        # starting thread to publish svg of bt
        self.vis_thread = threading.Thread(
            target=self.publish_svg_tree,
            daemon=True,
        )
        rospy.loginfo(f"starting svg tree publisher thread")
        self.vis_thread.start()

    def start_servers(self) -> None:
        # Service Servers for starting and stopping
        self.start_service = rospy.Service("hlp/start", Trigger, self.start_bt)
        self.stop_service = rospy.Service("hlp/stop", Trigger, self.stop_bt)
        self.pause_service = rospy.Service("hlp/pause", Trigger, self.pause_bt)

    def run_tree(self, hz: float = 30) -> None:
        """run tree in loop with pause support"""
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            if not self.is_running:
                break  # stop thread if not running

            if self.is_paused:
                rospy.logwarn_once("BT is paused. Waiting to continue...")
                rospy.sleep(0.1)  # small sleep to prevent CPU overload
                continue

            # self.update_state()  # pub hlp state for flask server
            self.tree.run(hz=hz, push_to_start=True, log_level="WARN")

    def start_bt(self, req: TriggerRequest) -> TriggerResponse:
        """start the BT execution"""
        if not self.tree.is_running():
            rospy.loginfo("Starting the BT...")
            self.tree.start()
            return TriggerResponse(success=True, message="BT started")

        return TriggerResponse(success=False, message="BT is already running.")

    def stop_bt(self, req: TriggerRequest) -> TriggerResponse:
        """stop and interrupt the BT"""

        rospy.loginfo("Stopping the BT...")
        self.tree.stop()

        if self.vis_thread is not None:
            self.vis_thread.join()
            self.vis_thread = None

        return TriggerResponse(success=True, message="BT stopped")

    def pause_bt(self, req: TriggerRequest) -> TriggerResponse:
        if self.tree.is_running():
            if self.tree.is_paused():
                rospy.loginfo("Continuing the BT.")
                self.tree.resume()
                return TriggerResponse(success=True, message=f"BT Continued.")
            else:
                rospy.loginfo("Pausing the BT.")
                self.tree.pause()
                return TriggerResponse(success=True, message=f"BT Paused.")

        return TriggerResponse(
            success=False,
            message="BT is not running, cannot pause/continue",
        )

    def update_state(self, tree, snapshot_visitor):
        state = String()
        tip_behaviour = tree.tip()
        state.data = tip_behaviour.name
        # state.data = pt.display.ascii_tree(
        #     tree.root, snapshot_information=snapshot_visitor
        # )

        self.hlp_status_pub.publish(state)

    def post_tick_handler(self, snapshot_visitor, tree):
        self.update_state(tree, snapshot_visitor)

