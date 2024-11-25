#!/usr/bin/env python

import visualizer as vis
import behaviours as bt

import rospy

import functools
import networkx as nx
import numpy as np
import py_trees
from typing import Tuple

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

import moma_utils.ros.gazebo_utils as gazebo_utils
import moma_utils.ros.transform_utils as utils


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


class PotholeBT:
    def __init__(self):

        self.load_parameters()

        self.root = py_trees.composites.Sequence(name="Pothole", memory=True)

        self.visualize_only = False

        self.init_move_to_pothole()
        self.init_deposit()
        self.init_sweeping()

        # self.root.add_children([self.sweep_1, self.sweep_2, self.sweep_3])
        self.root.add_children(
            [self.move_to_pothole, self.deposit, self.sweep_1]
        )

        #  self.root.add_children(sequence)

        self.tree = py_trees.trees.BehaviourTree(self.root)

    def pothole_cb(self, msg: Pose):
        # need some kind of error catching if goal not published!
        self.pothole_goal: Pose = msg

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def load_parameters(self):
        self.pothole_sub = rospy.Subscriber(
            "pothole_goal", Pose, self.pothole_cb, queue_size=10
        )
        self.current_pose = rospy.Subscriber(
            "odom", Odometry, self.odom_cb, queue_size=10
        )
        rospy.sleep(1.0)
        self.move_base_dist: float = rospy.get_param("~move_base_dist", 1.0)
        self.deposit_time: float = rospy.get_param("~deposit_time", 5.0)
        self.pothole_diameter: float = rospy.get_param("~pothole_diameter", 0.3)
        self.safe_pose = utils.find_parallel_pose(
            self.pothole_goal,
            self.current_pose,
            self.pothole_diameter * 2.0,
        )
        self.roller_up = True

    def init_move_to_pothole(self):
        mb_to_pothole = bt.MoveBase("movebase to pothole", self.safe_pose)
        move_to_pothole = bt.Move("move to pothole", "combined", self.safe_pose)
        at_safe_pos = bt.RobotAtPose("At safe pose?", self.safe_pose)

        self.at_safe = py_trees.composites.Selector(
            name="PrePotholePose", memory=True
        )
        self.at_safe.add_children([at_safe_pos, move_to_pothole])
        self.move_to_pothole = py_trees.composites.Sequence(
            name="MoveToPothole", memory=True
        )
        self.move_to_pothole.add_children([mb_to_pothole, self.at_safe])

    def init_deposit(self):

        forward_deposit = bt.Move(
            "move onto pothole",
            "forward",
            goal_distance=self.pothole_diameter * 2.0,
        )
        open_hatch = bt.TriggerComponent("open hatch", "hatch", "open")
        reverse_deposit = bt.Move(
            "move off pothole",
            "reverse",
            goal_distance=self.pothole_diameter * 2.0,
        )
        close_hatch = bt.TriggerComponent("close hatch", "hatch", "close")

        is_hatch_closed = bt.HatchUp("Is hatch closed?")
        is_hatch_open = py_trees.decorators.Inverter(
            is_hatch_closed, name="Is hatch open?"
        )

        open_hatch_sel = py_trees.Selector(name="OpenHatch", memory=True)
        open_hatch_sel.add_children([is_hatch_open, open_hatch])
        close_hatch_sel = py_trees.Selector(name="CloseHatch", memory=True)
        close_hatch_sel.add_children([is_hatch_closed, close_hatch])

        self.deposit = py_trees.composites.Sequence(name="Deposit", memory=True)
        self.deposit.add_children(
            [forward_deposit, open_hatch_sel, close_hatch_sel, reverse_deposit]
        )

    def init_sweeping(self):

        sweep_1, sweep_2, sweep_3 = self.calculate_sweeps(
            self.safe_pose, self.pothole_goal, self.pothole_diameter
        )

        move_sweep_1 = bt.Move("move to sweep 1", "combined", sweep_1)
        move_sweep_2 = bt.Move("move to sweep 2", "combined", sweep_2)
        move_sweep_3 = bt.Move("move to sweep 3", "combined", sweep_3)

        at_sweep_1 = bt.RobotAtPose("Ready for sweep 1?", sweep_1)
        at_sweep_2 = bt.RobotAtPose("Ready for sweep 2?", sweep_2)
        at_sweep_3 = bt.RobotAtPose("Ready for sweep 3?", sweep_3)

        forward_sweep = bt.Move(
            "sweep forward", "forward", goal_distance=self.pothole_diameter
        )
        backward_sweep = bt.Move(
            "sweep backward", "reverse", goal_distance=self.pothole_diameter
        )

        raise_roller = bt.TriggerComponent("raise roller", "roller", "raise")
        lower_roller = bt.TriggerComponent("lower roller", "roller", "lower")

        is_roller_up = bt.RollerUp("Is roller up?")
        is_roller_down = py_trees.decorators.Inverter(
            is_roller_up, "Is roller down?"
        )

        raise_sel = py_trees.composites.Selector(
            name="RaiseRoller", memory=True
        )
        raise_sel.add_children([is_roller_up, raise_roller])

        lower_sel = py_trees.composites.Selector(
            name="LowerRoller", memory=True
        )
        lower_sel.add_children([is_roller_down, lower_roller])

        sweep_1_sel = py_trees.composites.Selector(
            name="Sweep1Pos", memory=True
        )
        sweep_1_sel.add_children([at_sweep_1, move_sweep_1])

        sweep_2_sel = py_trees.composites.Selector(
            name="Sweep2Pos", memory=True
        )
        sweep_2_sel.add_children([at_sweep_2, move_sweep_2])

        sweep_3_sel = py_trees.composites.Selector(
            name="Sweep3Pos", memory=True
        )
        sweep_3_sel.add_children([at_sweep_3, move_sweep_3])

        self.sweep_1 = py_trees.composites.Sequence(name="Sweep1", memory=True)
        self.sweep_1.add_children(
            [sweep_1_sel, lower_sel, forward_sweep, raise_sel, backward_sweep]
        )

        self.sweep_2 = py_trees.composites.Sequence(name="Sweep2", memory=True)
        self.sweep_2.add_children(
            [sweep_2_sel, lower_sel, forward_sweep, raise_sel, backward_sweep]
        )

        self.sweep_3 = py_trees.composites.Sequence(name="Sweep3", memory=True)
        self.sweep_3.add_children(
            [sweep_3_sel, lower_sel, forward_sweep, raise_sel, backward_sweep]
        )

    def calculate_sweeps(
        self, start: Pose, pothole: Pose, diameter: float
    ) -> Tuple[Pose, Pose, Pose]:
        """
        finds 3 points along perpendicular line pose_a -> pose_b
        distance from line to pothole is diameter
        distance between points is diameter / 2

        returns 3 starting sweep poses
        """
        sweep_1 = utils.find_parallel_pose(
            pothole, start, diameter, towards=True
        )
        sweep_2 = utils.find_perpendicular_pose(
            sweep_1, pothole, diameter / 2.0, towards=True
        )
        sweep_3 = utils.find_perpendicular_pose(
            sweep_1, pothole, diameter / 2.0, towards=False
        )

        sweep_2.orientation = sweep_1.orientation
        sweep_3.orientation = sweep_1.orientation

        return sweep_1, sweep_2, sweep_3

    def visualize(self, bt_name: str):
        graph = nx.DiGraph(
            nx.drawing.nx_pydot.from_pydot(
                py_trees.display.generate_pydot_graph(self.tree.root, 1)
            )
        )
        print(f"{bt_name}, {graph}")

    def run(self):
        """The BT execution is visualized in the terminal."""
        self.tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.visitors.append(snapshot_visitor)
        self.tree.add_post_tick_handler(
            functools.partial(post_tick_handler, snapshot_visitor)
        )
        self.tree.setup(timeout=15)

        while not rospy.is_shutdown():
            rospy.Rate(3).sleep()
            self.tree.tick()
            if self.tree.root.status == py_trees.common.Status.SUCCESS:
                rospy.loginfo("Goal reached successfully")
                break

    def run_online(self):
        """The BT execution is visualized in a Chrome page that opens upon execution."""
        visualizer = vis.BTVisualizer(self.tree)

        self.tree.add_post_tick_handler(visualizer.update_graph)
        while not rospy.is_shutdown():
            rospy.Rate(3).sleep()
            # visulizer.tick()
            self.tree.tick()
            if self.tree.root.status == py_trees.common.Status.SUCCESS:
                rospy.loginfo("Goal reached successfully")
                break


def main():
    rospy.init_node("pothole_bt")

    goal_pose = Pose()
    goal_pose.position.x = 2.0
    goal_pose.position.y = -1.0
    goal_pose.orientation.z = 1.0

    rospy.loginfo("starting")
    node = PotholeBT()

    try:
        node.visualize("pothole")
        if not node.visualize_only:
            node.run_online()
    except rospy.ROSInterruptException as e:
        print(e)


if __name__ == "__main__":
    main()
