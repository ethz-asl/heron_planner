#!/usr/bin/env python

import visualizer as vis
import behaviours as bt

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


class PotholeBT:
    def __init__(self):

        # starting point, robot is near defect, have GPS of pothole
        # 
        # start inspection node -> srv call (go to pose in moveit)
        # find_pothole req to iccs_pothole_node
        # cont. if no pothole found
        # 
        # if pothole is found -> res CoM & surface area
        #
        # compute TF frame for pothole CoM in odom frame
        # 
        # send srv req to offset node for pothole res offset distance
        # send docking node srv req to docking_node, res done
        #
        # send cleaning req to command_manager/cleaning, res done
        # send docking req to docking_node res done
        # 
        # send surface_area req to deposite node res deposit seq (str)
        # send deposit seq req to command_manager/deposit res done
        # 
        # send docking req to docking node res done
        # 
        # wait for input from pilot GUI (topic or ROS param?)
        # execute roller sequence
        # monitor roller up or down
        # wait for pilot ok
        # 
        # send docking req to docking node res done
        #
        # send validation req for pothole
        # move arm, capture pic res image
        # 
        # send image to kafka
        # 
        # BEHAVIOURS
        # - inspection behaviour [MoveTo.action] #TODO define inspection poses
        # - docking beahviour [Dock.action] send goal_id (frame)
        # - req offset behaviour [offset_node tbd (res goal_id)]
        # - srv req behaviour//command_manager behaviour res success
        # - roller seq behaviour 
        # - validation behaviour
        # - kafka status behaviour
        # - kafka image behaviour 

        self.visualize_only = False
        self.is_running = False

        self.root = py_trees.composites.Sequence(
            name="MoveSequence", memory=True
        )
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

        self.tree = py_trees.trees.BehaviourTree(self.root)

        # Service Servers for starting and stopping
        self.start_service = rospy.Service("start_bt", Trigger, self.start_bt)
        self.stop_service = rospy.Service("stop_bt", Trigger, self.stop_bt)

    def start_bt(self, req: TriggerRequest): 
        """"""
        if self.is_running:
            return TriggerResponse(
                success=False, message="BT is already running"
            )
        self.is_running = True
        rospy.loginfo("Starting the BT...")
        visualizer = vis.BTVisualizer(self.tree)

        self.tree.add_post_tick_handler(visualizer.update_graph)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick_bt)
        return TriggerResponse(success=True, message="BT started")
    

    def stop_bt(self, req: TriggerRequest): 
        """"""
        if not self.is_running:
            return TriggerResponse(
                success=False, message="BT is not running"
            )
        self.is_running = False
        rospy.loginfo("Stopping the BT...")
        # rospy.Time(rospy.Duration(0.1), self.tick_bt)
        self.timer.shutdown()
        return TriggerResponse(success=True, message="BT stopped")
    

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
        if not self.is_running:
            return

        self.tree.tick()
        if self.tree.root.status == py_trees.common.Status.SUCCESS:
            rospy.loginfo("Goal reached successfully")
            self.is_running = False

def main():
    rospy.init_node("pothole_bt")

    node = PotholeBT()

    rospy.spin()


if __name__ == "__main__":
    main()
