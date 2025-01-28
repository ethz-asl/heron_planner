#!/usr/bin/env python

import heron_planner.utils.bt_rendering as render
import heron_planner.behaviours.component_behaviours as comp
import heron_planner.behaviours.base_behaviours as base
import heron_planner.behaviours.queue_behaviours as queue
import heron_planner.behaviours.arm_behaviours as arm
import heron_planner.behaviours.vision_behaviours as vision
import heron_planner.utils.simple_flask_server as flask_server
import threading
import requests
import functools

import rospy

import numpy as np
import py_trees as pt

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class PotholeBT:
    def __init__(self):
        self.visualize_only = False
        self.is_running = False
        self.is_paused = False
        self.previous_status = None
        self.target_inspection = ""

        self.load_parameters()
        self.load_subscribers()

        self.hlp_status_pub = rospy.Publisher(
            "hlp/state", String, queue_size=10
        )

        self.root = pt.composites.Sequence(
            name="InspectionSequence", memory=True
        )

        # arm home position
        at_home = arm.ArmAt("Arm at home?", "HOME")
        arm_to_home = arm.MoveTo("Move arm to home", "HOME")

        home_sel = pt.composites.Selector(
            "HomeSelector", children=[at_home, arm_to_home], memory=True
        )
        # pothole offset
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

        inspection_loop = pt.composites.Selector(
            name="Inspect and find pothole", memory=True
        )

        get_inspection = queue.GetLocationFromQueue(
            "Get inspection", "inspection_locations", "target_inspection"
        )
        at_inpection = arm.ArmAtDynamic(
            f"Arm at inspection?", self.fetch_next_pose
        )
        arm_to_inspection = arm.MoveToDynamic(
            f"Move to inspection", self.fetch_next_pose
        )
        inspect_sel = pt.composites.Selector(
            name=f"{self.target_inspection.lower()}Selector",
            children=[at_inpection, arm_to_inspection],
            memory=True,
        )
        find_pothole = vision.SearchForPothole(
            f"Find pothole at inspection",
            self.image_rgb,
            self.image_depth,
            self.camera_info,
        )

        inspect_location = pt.composites.Sequence(
            name="Search Inspection Location",
            memory=False,
            children=[get_inspection, inspect_sel, find_pothole],
        )

        retry_inspections = queue.RetryUntilSuccessful(
            child=inspect_location,
            max_attempts=len(self.inspection_names),
            name="Retry Inspection Locations",
        )

        no_pothole_found = pt.behaviours.Failure(name="No pothole found")
        inspection_loop.add_children([retry_inspections, no_pothole_found])

        send_pothole_to_kakfa = vision.SendPhotoToKafka(
            "Send pothole segmention to Kafka"
        )

        # here get the pose and offset
        pothole_odom_frame = base.PoseTransformer(
            "Pothole in odom frame", self.pothole_com, "odom", "pothole/pose" 
        )
        find_pothole_offset = base.OffsetFinder(
            "Find pothole offset", self.pothole_pose, "POTHOLE", "pothole/offset"
        )

        # pothole offset
        at_pothole_offset = base.AtPose(
            "At pothole offset?", self.pothole_offset
        )
        go_to_pothole_offset = base.GoToGPS(
            "Go to pothole offset", self.pothole_offset
        )
        offset_sel = pt.composites.Selector(
            "PotholeOffsetSelector",
            children=[at_pothole_offset, go_to_pothole_offset],
            memory=True,
        )

        offset_seq = pt.composites.Sequence(
            name="GetToOffset",
            children=[pothole_odom_frame, find_pothole_offset, offset_sel]
        )

        offset_par = pt.composites.Parallel(
            name="OffsetParallel",
            children=[send_pothole_to_kakfa, offset_seq]
        )

        inspection_mode = pt.composites.Sequence(
            name="Inspect for pothole",
            memory=True,
            children=[inspection_loop, offset_par],
        )

        self.root.add_children([init_seq, inspection_mode])

        self.tree = pt.trees.BehaviourTree(self.root)

        snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.publish_to_html(self.tree, snapshot_visitor)

        # starting simple flask server thread to serve svg of bt
        rospy.loginfo("starting flask server...")
        self.vis_thread = threading.Thread(
            target=flask_server.start_flask_server, args=(self.tree,)
        )
        self.vis_thread.daemon = True
        self.vis_thread.start()

        # Service Servers for starting and stopping
        self.start_service = rospy.Service("start_bt", Trigger, self.start_bt)
        self.stop_service = rospy.Service("stop_bt", Trigger, self.stop_bt)
        self.pause_service = rospy.Service("pause_bt", Trigger, self.pause_bt)

    def load_parameters(self) -> None:
        self.pothole_offset_gps = NavSatFix(
            header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
            latitude=35,
            longitude=-125,
            altitude=0,
        )
        self.pothole_start = PoseStamped()
        self.pothole_pose = PoseStamped()
        self.pothole_com = PoseStamped()
        self.pothole_offset = PoseStamped()
        self.inspection_names = [
            "INSPECTION1",
            "INSPECTION2",
            "INSPECTION3",
            "INSPECTION4",
            "INSPECTION5",
        ]
        rospy.set_param("inspection_locations", self.inspection_names)

    def fetch_next_pose(self) -> None:
        return self.target_inspection

    def load_subscribers(self) -> None:
        self.dummy_imgs()
        self.pothole_start_sub = rospy.Subscriber(
            "/robot/pothole_start", PoseStamped, self.pothole_start_cb
        )
        self.target_inspection_sub = rospy.Subscriber(
            "target_inspection", String, self.target_inspection_cb
        )
        self.pothole_com_sub = rospy.Subscriber(
            "pothole/center_of_mass", PoseStamped, self.pothole_pose_cb
        )
        self.pothole_pose_sub = rospy.Subscriber(
            "pothole/pose", PoseStamped, self.pothole_pose_cb
        )
        self.pothole_offset_sub = rospy.Subscriber(
            "pothole/offset", PoseStamped, self.pothole_offset_cb
        )

    def pothole_start_cb(self, msg: PoseStamped):
        self.pothole_start = msg
        # rospy.logwarn(f"pothole_start: {self.pothole_start}")

    def target_inspection_cb(self, msg: String) -> None:
        self.target_inspection = msg.data
        rospy.logwarn(f"target_inspection: {self.target_inspection}")

    def pothole_com_cb(self, msg: PoseStamped):
        self.pothole_com = msg
        rospy.logwarn(f"pothole_com: {self.pothole_com}")

    def pothole_pose_cb(self, msg: PoseStamped):
        self.pothole_pose = msg
        rospy.logwarn(f"pothole_pose: {self.pothole_pose}")

    def pothole_offset_cb(self, msg: PoseStamped):
        self.pothole_offset = msg
        rospy.logwarn(f"pothole_pose: {self.pothole_offset}")

    def dummy_imgs(self) -> None:
        # Populate dummy request data
        self.image_rgb = Image()
        self.image_rgb.height = 480
        self.image_rgb.width = 640
        self.image_rgb.encoding = "rgb8"
        self.image_rgb.step = 1920
        self.image_rgb.data = []

        self.image_depth = Image()
        self.image_depth.height = 480
        self.image_depth.width = 640
        self.image_depth.encoding = "32FC1"
        self.image_depth.step = 2560
        self.image_depth.data = []

        self.camera_info = CameraInfo()
        self.camera_info.height = 480
        self.camera_info.width = 640

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
        state = pt.display.ascii_tree(
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
