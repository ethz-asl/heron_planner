#!/usr/bin/env python

import heron_planner.behaviours.base_behaviours as base
import heron_planner.behaviours.arm_behaviours as arm
import heron_planner.behaviours.leaf_behaviours as leaf
import heron_planner.utils.simple_flask_server as flask_server
import heron_planner.utils.bt_runner as bt_runner
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

        self.load_parameters()
        self.load_subscribers()

        self.root = self.build_root()
        self.tree = bt_runner.BehaviourTreeRunner("TestTree", self.root)
        self.start_flask_server()

        self.hlp_status_pub = rospy.Publisher(
            "hlp/state", String, queue_size=10
        )

        self.start_servers()

    def build_root(self):
        """build root"""

        root = pt.composites.Sequence(name="InspectionSequence", memory=True)

        ####################################################################
        # saving data to blackboard
        ####################################################################
        save_start = leaf.SaveData(data=self.pothole_start, save_key="start")
        save_inspections = leaf.SaveData(data=self.inspection_names, save_key="inspections")

        ####################################################################
        # inialtial positions
        ####################################################################
        # arm home position
        at_home = leaf.ArmAt(task_name="Arm at home?", pose_name="HOME")
        arm_to_home = leaf.MoveTo(
            task_name="Move arm to home", load_value="HOME"
        )

        home_sel = pt.composites.Selector(
            "HomeSelector", children=[at_home, arm_to_home], memory=True
        )

        # base at start
        at_pothole_start = leaf.AtPose(
            task_name="At pothole start?", pose_name="start"
        )
        go_to_pothole_start = leaf.GoToGPS(
            task_name="Go to pothole start", load_key="start"
        )

        pothole_sel = pt.composites.Selector(
            "PotholeStartSelector",
            children=[at_pothole_start, go_to_pothole_start],
            memory=True,
        )

        init_seq = pt.composites.Sequence(
            name="Before inspection",
            children=[save_start, home_sel, pothole_sel],
            memory=True,
        )

        ####################################################################
        # loop through inspection positons and find pothole
        ####################################################################
        inspection_loop = pt.composites.Selector(
            name="Inspect and find pothole", memory=True
        )

        get_inspection = leaf.PopFromList(
            task_name="Get inspection", load_key="inspections", save_key="inspection"
        )
        at_inspection = leaf.ArmAt(
            task_name="Arm at inspection?", pose_name="inspection"
        )
        arm_to_inspection = leaf.MoveTo(
            task_name="Move arm to inspection.", load_key="inspection"
        )
        inspection_sel = pt.composites.Selector(
            name="InspectionSelector",
            children=[at_inspection, arm_to_inspection],
            memory=True
        )
        
        #TODO add load current photo
        #TODO add srv call to find pothole


        wait_for_enter = leaf.WaitForEnterKey()

        root.add_children([init_seq, wait_for_enter])

        return root
        # return rt.trees.BehaviourTree("TestingLeaves", root)

        # self.tree = rt.trees.BehaviourTree(
        #     "TestingLeaves",
        #     pt.composites.Sequence(
        #         name="InspectionSequence",
        #         children=[
        #            pt.composites.Sequence(
        #                name="BeforeInspection",
        #                children=[
        #                    save_start, home_sel, pothole_sel
        #                ]
        #            )
        #         ]
        #     )
        # )

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

    def load_parameters(self) -> None:

        self.pothole_start = PoseStamped()
        self.pothole_start.pose.position.x = 10.0
        self.pothole_start.pose.position.y = 5.0
        self.pothole_start.pose.orientation.z = 0.707
        self.pothole_start.pose.orientation.w = 0.707

        self.tree_rate = rospy.get_param("tree_rate", 10)

        self.inspection_names = [
            "INSPECTION1",
            "INSPECTION2",
            "INSPECTION3",
            "INSPECTION4",
            "INSPECTION5",
        ]

    def load_subscribers(self) -> None:
        pass

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

    def start_bt(self, req) -> TriggerResponse:
        """start the BT execution"""
        if not self.tree.is_running():
            rospy.loginfo("Starting the BT...")
            self.tree.start()
            return TriggerResponse(success=True, message="BT started")
            
        return TriggerResponse(
            success=False, message="BT is already running."
        )

    def stop_bt(self, req) -> TriggerResponse:
        """stop and interrupt the BT"""

        rospy.loginfo("Stopping the BT...")
        self.tree.stop()

        if self.vis_thread is not None:
            self.vis_thread.join()
            self.vis_thread = None

        return TriggerResponse(success=True, message="BT stopped")

    def pause_bt(self, req) -> TriggerResponse:
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
        self.update_state(tree, snapshot_visitor)


def main():
    rospy.init_node("pothole_leaves_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = PotholeBT()
    node.tree.visualise()
    # node.tree.run(hz=30, push_to_start=True, log_level="WARN")

    rospy.spin()


if __name__ == "__main__":
    main()
