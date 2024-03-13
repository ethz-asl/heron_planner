#!/usr/bin/env python

import mobile_manip_demo.behaviors as bt
from mobile_manip_demo.environment import get_place_pose
from mobile_manip_demo.visualizer import BTVisualizer

import rospy

import functools
import networkx as nx
import numpy as np
import py_trees
import random


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


class HeronBT:
    def __init__(self):
        """Initialize ROS nodes."""
        print("Initialising the HeronBT node")
        cone_ID = 0 #2
        # Parameters
        cone_pickup_target = rospy.get_param("heron_demo/cone_pickup")
        place_target = rospy.get_param("heron_demo/defect_zone")
        place_pose = get_place_pose(np.array(cone_pickup_target), np.array(place_target))
        safe_pose = rospy.get_param("heron_demo/safe_zone")

        task_type = rospy.get_param("heron_demo/experiment")
        self.visualization_only = rospy.get_param("heron_demo/visualization_only")

        # Mobile Picking:
        move_to_pick = py_trees.composites.Selector(name="Fallback")
        
        print("Added selector")

        move_to_pick.add_children(
            [
                bt.RobotAtPose(
                    name=f"Is robot at cone pickup location?",
                    robot_name="panda",
                    pose=cone_ID,
                    tolerance=0.1,
                ),
                bt.Move(name=f"Move to cone pickup location", goal_ID=cone_ID),
            ]
        )
        print("Added child")

        pick_sequence = bt.RSequence(name="Sequence")
        pick_sequence.add_children(
            [
                move_to_pick,
                bt.Pick(name=f"Pick up cone", goal_ID=cone_ID),
            ]
        )
        pick = py_trees.composites.Selector(name="Fallback")
        pick.add_children(
            [
                bt.InHand(name=f"Is cone secured in gripper?"),
                pick_sequence,
            ]
        )

        # Mobile Placing:
        move_to_place = py_trees.composites.Selector(name="Fallback")
        move_to_place.add_children(
            [
                bt.RobotAtPose(
                    name=f"Is robot at defect location?",
                    robot_name="panda",
                    pose=np.array(cone_pickup_target),
                    tolerance=0.15,
                ),
                bt.Move(name=f"Move to defect location", goal_pose=np.array(cone_pickup_target)),
            ]
        )

        place_sequence = bt.RSequence(name="Sequence")
        place_sequence.add_children(
            [
                pick,
                move_to_place,
                bt.Place(
                    name=f"Place cone",
                    goal_ID=cone_ID,
                    goal_pose=place_target,
                ),
            ]
        )

        # MoMa
        moma = py_trees.composites.Selector(name="Fallback")
        moma.add_children(
            [
                bt.RobotAtPose(
                    name=f"Is cone at defect location?",
                    robot_name="panda",
                    pose=place_pose,
                    tolerance=0.5,
                ),
                place_sequence,
            ]
        )

        # Recharge
        recharge = py_trees.composites.Selector(name="Fallback")
        recharge.add_children(
            [
                bt.BatteryLv(
                    name="Battery > 20%?",
                    relation="greater",
                    value=20.0,
                ),
                bt.Recharge(name="Recharge"),
            ]
        )

        # Dock
        safe = py_trees.composites.Selector(name="Fallback")
        safe.add_children(
            [
                bt.RobotAtPose(
                    name=f"Is robot at safe location?",
                    robot_name="panda",
                    pose=np.array(safe_pose),
                    tolerance=0.1,
                ),
                bt.Move(name="Move to safe location", goal_pose=np.array(safe_pose)),
            ]
        )

        self.root = bt.RSequence(name="Sequence")
        if task_type == 2:
            self.root.add_children([recharge, moma])
        elif task_type == 3:
            self.root.add_children([recharge, moma, safe])
        else:
            self.root = moma

        self.tree = py_trees.trees.BehaviourTree(self.root)

    def get_root(self) -> py_trees.composites.Selector:
        return self.root

    def visualize(self, bt_name: str):
        """Compute the number of nodes and transition in a BT and save it as figure."""
        py_trees.display.render_dot_tree(self.tree.root, name=bt_name)
        graph = nx.DiGraph(
                nx.drawing.nx_pydot.from_pydot(py_trees.display.generate_pydot_graph(self.tree.root, 1))
        )
        print(f"{bt_name}, {graph}")

    def run(self):
        """The BT execution is visualized in the terminal."""
        self.tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.tree.setup(timeout=15)

        while not rospy.is_shutdown():
            rospy.Rate(1).sleep()
            self.tree.tick()

    def run_online(self):
        """The BT execution is visualized in a Chrome page that opens upon execution."""
        viz = BTVisualizer(self.tree)

        self.tree.add_post_tick_handler(viz.update_graph)
        while not rospy.is_shutdown():
            rospy.Rate(1).sleep()
            # viz.tick()
            self.tree.tick()


def main():
    rospy.init_node("BehaviorTree")
    node = HeronBT()

    try:
        node.visualize("heron")
        if not node.visualization_only:
            node.run_online()
        pass
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
