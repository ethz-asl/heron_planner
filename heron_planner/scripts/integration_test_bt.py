#!/usr/bin/env python

import visualizer as vis
import component_behaviours as comp_bt
import base_behaviours as base_bt
import arm_behaviours as arm_bt

import rospy

import functools
import networkx as nx
import numpy as np
import py_trees

from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


def post_tick_handler(snapshot_visitor, behavior_tree):
    """Prints an ascii tree with the current snapshot status."""
    print(
        "\n"
        + py_trees.display.unicode_tree(
            root=behavior_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )
    name_ = "root" + str(behavior_tree.count)


class IntegrationTestBT:
    def __init__(self):

        self.visualize_only = False
        self.is_running = False
        self.is_paused = False

        self.root = py_trees.composites.Sequence(
            name="TestSequence", memory=True
        )

        activate_blower = comp_bt.Blow("Blow pothole")
        lower_roller = comp_bt.LowerRoller("Lower roller")
        lift_roller = comp_bt.LiftRoller("Lift roller")

        open_deposit_1 = comp_bt.OpenDeposit("Open deposit 1", "1")
        open_deposit_2 = comp_bt.OpenDeposit("Open deposit 2", "2")
        open_deposit_3 = comp_bt.OpenDeposit("Open deposit 3", "3")

        move_forward_5cm = base_bt.Move("Move forward 5cm", "0.05", "0.0")
        move_right_5cm = base_bt.Move("Move left 5cm", "0.0", "0.05")
        turn_a_bit = base_bt.Turn("Turn 0.3 rad", "0.3")

        wait_10s = base_bt.Wait("Wait 10s", "10")
        wait_10s_ros = base_bt.ROSWait("Wait 10s", 10)

        self.roller = py_trees.composites.Sequence(
            name="RollerSequence", memory=True
        )
        self.roller.add_children([lower_roller, wait_10s, lift_roller])

        self.blow_seq = py_trees.composites.Sequence(
            name="BlowSeq", memory=True
        )
        self.blow_seq.add_children(
            [activate_blower, wait_10s, move_forward_5cm, wait_10s]
        )

        # self.wait = py_trees.composites.Sequence(name="wait", memory=True)
        # self.wait.add_children([wait_10s_ros, lower_roller])

        self.root.add_children([self.blow_seq])

        self.tree = py_trees.trees.BehaviourTree(self.root)

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
        visualizer = vis.BTVisualiser(self.tree)

        self.tree.add_post_tick_handler(lambda tree: visualizer.update_graph(tree))
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

    def visualize(self, bt_name: str):
        """Compute the number of nodes and transition in a BT and save it as figure."""
        # py_trees.display.render_dot_tree(self.tree.root, name=bt_name)
        graph = nx.DiGraph(
            nx.drawing.nx_pydot.from_pydot(
                py_trees.display.generate_pydot_graph(self.tree.root, 1)
            )
        )
        print(f"{bt_name}, {graph}")

    def tick_bt(self, event):
        """"""
        # rospy.logwarn(f"Ticking BT: is_running={self.is_running}, is_paused={self.is_paused}")
        if not self.is_running:
            return

        if self.is_paused:
            rospy.loginfo_once("BT is paused. Waiting to continue...")
            return
        
        self.tree.tick()

        if self.tree.root.status == py_trees.common.Status.SUCCESS:
            rospy.loginfo("BT finished successfully")
            self.is_running = False
            self.timer.shutdown()


def main():
    rospy.init_node("integration_test_bt")
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    node = IntegrationTestBT()

    rospy.spin()


if __name__ == "__main__":
    main()
