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


class PotholeBT:
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

    def save_to_blackboard(self) -> None:
        ####################################################################
        # saving data to blackboard
        ####################################################################
        self.bb.set("start", self.pothole_start)
        self.bb.set("inspections", self.inspection_names)
        self.bb.set("arm_cam_ns", self.arm_cam_ns)
        self.bb.set("body_cam_ns", self.body_cam_ns)

    def load_parameters(self) -> None:

        self.pothole_start = PoseStamped()
        self.pothole_start.pose.position.x = 10.0
        self.pothole_start.pose.position.y = 5.0
        self.pothole_start.pose.orientation.z = 0.707
        self.pothole_start.pose.orientation.w = 0.707

        self.tree_rate = rospy.get_param("tree_rate", 10)

        self.inspection_names = rospy.get_param("ugv/inspection_names")

        self.body_cam_ns = rospy.get_param(
            "ugv/body_cam_ns", "/robot/base_camera/front_rgbd_camera/"
        )
        self.arm_cam_ns = rospy.get_param(
            "ugv/arm_cam_ns", "/robot/arm_camera/front_rgbd_camera/"
        )

    def get_initial_seq(self) -> pt.composites.Composite:
        ####################################################################
        # initial positions
        ####################################################################
        # arm home position
        arm_to_home = ugv.MoveTo(
            task_name="Move arm to home", load_value="HOME"
        )

        # base at start
        at_pothole_start = ugv.AtPose(
            task_name="At pothole start?", pose_name="start"
        )
        go_to_pothole_start = ugv.GoToGPS(
            task_name="Go to pothole start", load_key="start"
        )

        pothole_sel = pt.composites.Selector(
            "PotholeStartSelector",
            children=[at_pothole_start, go_to_pothole_start],
            memory=True,
        )

        init_seq = pt.composites.Sequence(
            name="Before inspection",
            children=[arm_to_home, pothole_sel],
            memory=True,
        )
        return init_seq

    def get_inspection_seq(self) -> pt.composites.Composite:
        ####################################################################
        # loop through inspection positons and find pothole
        ####################################################################
        inspection_loop = pt.composites.Selector(
            name="Inspect and find pothole", memory=True
        )

        get_inspection = generic.PopFromList(
            task_name="Get inspection",
            load_key="inspections",
            save_key="inspection",
        )
        arm_to_inspection = ugv.MoveTo(
            task_name=f"Move arm inspection.", load_key="inspection"
        )

        load_arm_img_inspection = hlp.GetSynchedImages(
            task_name="Load wrist image",
            load_key="arm_cam_ns",
            image_key="pothole/rgb/before",
            save=True,
        )
        find_pothole = iccs.FindPothole(task_name="Find pothole at inspection")

        find_pothole_seq = pt.composites.Sequence(
            name="FindPotholeSeq",
            children=[load_arm_img_inspection, find_pothole],
            memory=False,
        )

        inspect_location = pt.composites.Sequence(
            name="Search Inpsection Location",
            memory=False,
            children=[get_inspection, arm_to_inspection, find_pothole_seq],
        )

        retry_inspection = generic.RetryUntilSuccessful(
            child=inspect_location,
            max_attempts=len(self.inspection_names),
            name="Retry Inspection Locations",
        )
        no_pothole_found = pt.behaviours.Failure(name="No pothole found")
        inspection_loop.add_children([retry_inspection, no_pothole_found])

        ####################################################################
        # if pothole found, send to kafka
        ####################################################################
        send_inspection_to_kafka = hlp.SendImageToKafka(
            task_name="Send inspection to Kafka",
            msg="Fake image for HLP testing",
            load_key="pothole/rgb/before",
        )
        inspection_seq = pt.composites.Sequence(
            name="InpsectionSeq",
            children=[inspection_loop, send_inspection_to_kafka],
        )

        return inspection_seq

    def get_offset_seq(self) -> pt.composites.Composite:
        ####################################################################
        # find safe offset
        ####################################################################
        pothole_in_odom = hlp.TransformPose(
            task_name="Transform pothole CoM to odom",
            target_frame="odom",
            load_key="/pothole/com",
            save_key="/pothole/odom",
        )

        find_offset = ugv.FindOffset(
            task_name="Find pothole offset",
            defect="POTHOLE",
            broadcast=True,
            broadcast_frame="pothole_offset",
            load_key="/pothole/odom",
            save_key="/pothole/offset",
        )
        offset_seq = pt.composites.Sequence(
            name="OffsetSeq",
            children=[pothole_in_odom, find_offset],  # add dock
            memory=True,
        )
        return offset_seq

    def get_offset_sel(self) -> pt.composites.Composite:
        ####################################################################
        # dock to offset location
        ####################################################################

        at_offset = ugv.AtPose(
            task_name="At offset?", pose_name="/pothole/offset"
        )
        dock_to_offset = ugv.Dock(
            task_name="Dock to offset", load_value="pothole_offset"
        )
        offset_sel = pt.composites.Selector(
            name="OffsetSel", children=[at_offset, dock_to_offset], memory=True
        )
        return offset_sel

    def get_deposit_seq(self) -> pt.composites.Composite:
        ####################################################################
        # get deposit sequence
        ####################################################################

        get_deposit = ugv.GetDepositSeq(
            task_name="Get deposit sequence", load_key="/pothole/surface_area"
        )

        deposit_material = ugv.Deposit()

        deposit_seq = pt.composites.Sequence(
            name="DepositSeq",
            children=[get_deposit, deposit_material],
            memory=True,
        )
        return deposit_seq

    def get_validation_seq(self) -> pt.composites.Composite:
        arm_to_validation = ugv.MoveTo(
            task_name="Move arm to validation", load_value="VALIDATION"
        )

        load_arm_img_validation = hlp.GetSynchedImages(
            task_name="Load wrist image",
            load_key="arm_cam_ns",
            image_key="pothole/rgb/after",
            save=True,
        )
        send_validation_to_kafka = hlp.SendImageToKafka(
            task_name="Send validation to Kafka",
            msg="Fake image for HLP testing",
            load_key="pothole/rgb/after",
        )

        validation_seq = pt.composites.Sequence(
            name="ValidationSeq",
            children=[
                arm_to_validation,
                load_arm_img_validation,
                send_validation_to_kafka,
            ],
        )
        return validation_seq

    def build_root(self):
        """build root"""

        root = pt.composites.Sequence(name="InspectionSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        # TODO pilot integration (bt_ui & service calls)
        # TODO Roller sequence

        # dock_to_offset again!

        root.add_children(
            [
                self.get_initial_seq(),
                self.get_inspection_seq(),
                self.get_offset_seq(),
                self.get_offset_sel(),
                self.get_deposit_seq(),
                self.get_validation_seq(),
                self.get_offset_sel(),
                wait_for_enter,
            ]
        )

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
        tip_behaviour = tree.tip()
        state.data = tip_behaviour.name
        state.data = pt.display.ascii_tree(
            tree.root, snapshot_information=snapshot_visitor
        )

        self.hlp_status_pub.publish(state)

    def post_tick_handler(self, snapshot_visitor, tree):
        self.update_state(tree, snapshot_visitor)


def main():
    rospy.init_node("pothole_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = PotholeBT()
    node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
