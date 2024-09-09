#!/usr/bin/env python

import visualizer as vis
import behaviours as bt

import rospy

import functools
import networkx as nx
import numpy as np
import py_trees

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

import utils.transform_utils as utils

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

        self.root = py_trees.composites.Sequence(
            name="MoveSequence", memory=True
        )
        self.visualize_only = False
        move_base = bt.MoveBase("movebase to pothole", self.safe_pose)
        move = bt.Move("move to pothole", "combined", self.safe_pose)
        move_forward = bt.Move("move onto pothole", "forward", goal_distance=self.pothole_diameter * 2.0)
        move_reverse = bt.Move("move off pothole", "reverse", goal_distance=self.pothole_diameter * 2.0)
        move_right = bt.Move("move right", "right", goal_distance=0.8)
        move_left = bt.Move("move left", "left", goal_distance=1.2)
        move_cw = bt.Move("move cw", "cw", goal_distance=np.deg2rad(180))
        move_ccw = bt.Move("move ccw", "ccw", goal_distance=np.deg2rad(45))

        self.move_to_pothole = py_trees.composites.Sequence(name="MoveToPothole", memory=True)
        self.move_to_pothole.add_children([move_base, move])

        self.dumping = py_trees.composites.Sequence(name="Dumping", memory=True)
        self.dumping.add_children([move_forward, move_reverse])

        self.root.add_children([self.move_to_pothole, self.dumping])

        #  self.root.add_children(sequence)

        self.tree = py_trees.trees.BehaviourTree(self.root)

    #  def get_root(self) -> py_trees.composites.Selector:
    #  return self.root
    def pothole_cb(self, msg: Pose):
        self.pothole_goal: Pose = msg

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def load_parameters(self):
        self.pothole_sub = rospy.Subscriber("pothole_goal", Pose, self.pothole_cb, queue_size=10)
        self.current_pose = rospy.Subscriber("odom", Odometry, self.odom_cb, queue_size=10)
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

    forward_dist = 0.5
    reverse_dist = 1.0
    
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
