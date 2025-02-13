#!/usr/bin/env python

import heron_planner.behaviours.ugv_behaviours as ugv
import heron_planner.behaviours.hlp_behaviours as hlp
import heron_planner.behaviours.iccs_behaviours as iccs
import heron_planner.behaviours.generic_behaviours as generic

import heron_utils.simple_flask_server as flask_server
import heron_utils.bt_runner as bt_runner
import functools

import rospy
import threading

import py_trees as pt
import ros_trees as rt

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class ConePlaceBT:
    def __init__(self) -> None:
        self.visualize_only = False
        self.is_running = False
        self.is_paused = False
        self.previous_status = None

        self.bb = pt.Blackboard()
        self.load_parameters()
        self.save_to_blackboard()

        self.root = self.build_root()
        self.tree = bt_runner.BehaviourTreeRunner("TestTree", self.root)
        self.start_flask_server()

        self.hlp_status_pub = rospy.Publisher(
            "hlp/state", String, queue_size=10
        )

        self.start_servers()

    def load_parameters(self) -> None:

        self.tree_rate = rospy.get_param("tree_rate", 10)

        self.inspection_names = rospy.get_param("ugv/inspection_names")

        self.body_cam_ns = rospy.get_param(
            "ugv/body_cam_ns", "/robot/base_camera/front_rgbd_camera/"
        )
        self.arm_cam_ns = rospy.get_param(
            "ugv/arm_cam_ns", "/robot/arm_camera/front_rgbd_camera/"
        )

    def save_to_blackboard(self) -> None:
        ####################################################################
        # saving data to blackboard
        ####################################################################
        self.bb.set("arm_cam_ns", self.arm_cam_ns)
        self.bb.set("body_cam_ns", self.body_cam_ns)

    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="InspectionSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        arm_to_home = ugv.MoveTo(
            task_name="Move arm to home", load_value="HOME"
        )

        arm_to_inspection = ugv.MoveTo(
            task_name=f"Move arm inspection.", load_key="inspection"
        )
        inspect_seq = pt.composites.Sequence(
            name="InspectionSelector",
            children=[arm_to_home, arm_to_inspection],
            memory=True,
        )

        load_arm_img_inspection = hlp.GetSynchedImages(
            task_name="Load wrist image",
            load_key="arm_cam_ns",
            image_key="cone/rgb",
            save=True,
        )

        # KAFKA BROKER DOWN
        # send_inspection_to_kafka = hlp.SendImageToKafka(
        #     task_name="Send inspection to Kafka",
        #     msg="Real image for HLP testing",
        #     load_key="cone/rgb",
        # )
        send_inspection_to_kafka = generic.Wait(
            task_name="Send inspection to Kafka", duration=5
        )

        take_photo_seq = pt.composites.Sequence(
            name="PhotoSequence",
            children=[load_arm_img_inspection, send_inspection_to_kafka],
            memory=True,
        )

        root.add_children([inspect_seq, take_photo_seq])

        return root

    def start_flask_server(self) -> None:
        # to update svg
        snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)

        # starting simple flask server thread to serve svg of bt
        self.vis_thread = threading.Thread(
            target=flask_server.start_flask_server,
            args=(self.tree,),
            daemon=True,
        )
        rospy.loginfo("starting flask server...")
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
        # tip_behaviour = tree.tip()
        # state.data = tip_behaviour.name
        state.data = pt.display.ascii_tree(
            tree.root, snapshot_information=snapshot_visitor
        )

        self.hlp_status_pub.publish(state)

    def post_tick_handler(self, snapshot_visitor, tree):
        self.update_state(tree, snapshot_visitor)


def main():
    rospy.init_node("cone_place_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = ConePlaceBT()
    node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
