#!/usr/bin/env python

import visualizer as vis
import behaviours as bt

import rospy

import functools
import networkx as nx
import numpy as np
import py_trees

from geometry_msgs.msg import Pose


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


class TaskBT:
    def __init__(self, goal_pose: Pose):

        # TODO add this
        # rospy.get_param("~goal_pose")
        #  self.root = bt.RSequence(name="Sequence")

        self.root = py_trees.composites.Sequence(
            name="MoveSequence", memory=True
        )
        self.visualize_only = False
        move_base = bt.MoveBase("move_base to goal", goal_pose)
        move = bt.Move("move to goal", "combined", goal_pose)
        move_forward = bt.Move("move forward", "forward", goal_distance=0.5)
        move_reverse = bt.Move("move backwards", "reverse", goal_distance=1.0)
        move_right = bt.Move("move right", "right", goal_distance=0.8)
        move_left = bt.Move("move left", "left", goal_distance=1.2)
        move_cw = bt.Move("move cw", "cw", goal_distance=np.deg2rad(180))
        move_ccw = bt.Move("move ccw", "ccw", goal_distance=np.deg2rad(45))

        self.move_to_goal = py_trees.composites.Sequence(name="MoveToGoal", memory=True)
        self.move_to_goal.add_children([move_base, move])

        self.move_around = py_trees.composites.Sequence(name="MoveAround", memory=True)
        self.move_around.add_children([move_right, move_cw, move_left, move_ccw])

        self.root.add_children([self.move_to_goal, self.move_around])

        #  self.root.add_children(sequence)

        self.tree = py_trees.trees.BehaviourTree(self.root)

    #  def get_root(self) -> py_trees.composites.Selector:
    #  return self.root

    def visualize(self, bt_name: str):
        """Compute the number of nodes and transition in a BT and save it as figure."""
        # py_trees.display.render_dot_tree(self.tree.root, name=bt_name)
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
    rospy.init_node("move_bt")

    goal_pose = Pose()
    goal_pose.position.x = 2.0
    goal_pose.position.y = -1.0
    goal_pose.orientation.z = 1.0

    forward_dist = 0.5
    reverse_dist = 1.0
    
    rospy.loginfo("Sending goal to robot")
    node = TaskBT(goal_pose)

    try:
        node.visualize("move")
        if not node.visualize_only:
            node.run_online()
    except rospy.ROSInterruptException as e:
        print(e)


if __name__ == "__main__":
    main()
