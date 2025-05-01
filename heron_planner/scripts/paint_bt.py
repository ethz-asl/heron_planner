#!/usr/bin/env python

import rospy

import py_trees as pt
import ros_trees as rt

import heron_planner.trees.base_bt as base_bt

import heron_planner.leaves.ugv_behaviours as ugv
import heron_planner.leaves.hlp_behaviours as hlp
import heron_planner.leaves.iccs_behaviours as iccs
import heron_planner.leaves.generic_behaviours as generic

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PaintBT(base_bt.BaseBT):
    def __init__(self) -> None:
        super().__init__("PaintBT")
        
    def load_parameters(self) -> None:
        self.tree_rate = rospy.get_param("tree_rate", 10)
        self.body_cam_ns = rospy.get_param(
            "/ugv/body_cam_ns", "/robot/body_camera"
        )
        self.arm_cam_ns = rospy.get_param("/ugv/arm_cam_ns", "/robot/arm_camera")
        self.arm_cam_tf = rospy.get_param("/ugv/arm_cam_tf", "")
        self.use_kafka = rospy.get_param("/kafka", False)

    def save_to_blackboard(self) -> None:
        self.bb.set("arm_cam_ns", self.arm_cam_ns)
        self.bb.set("body_cam_ns", self.body_cam_ns)

    def move_take_snap(
        self, move_loc: str = "home", seq_task_name: str = "MoveToHomeSeq"
    ) -> pt.composites.Composite:
        move_arm = ugv.MoveArmTo(
            task_name=f"Move arm to {move_loc}", load_value=move_loc
        )

        take_snap = ugv.TakeSnap()
        return pt.composites.Sequence(
            name=seq_task_name, children=[move_arm, take_snap], memory=True
        )


    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="PaintSequence", memory=True)
        arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to change tool position", load_value="change_tool_position"
        )
        arm_to_paint = ugv.MoveArmTo("Move arm to painting position", load_value="painting")
        
        # wait for TF follower to start (or not?)
        wait_for_completion = generic.WaitForEnterKey(task_name="Has TF path been completed?")
        
        move_forward = ugv.Move("Move forward", load_value="MOVE 5.0 0")
        wait_until_paint = generic.Wait(task_name="Wait to start painting", duration=0.1)
        start_paint = ugv.PaintOn()
        wait_after_paint = generic.Wait(task_name="Wait to stop painting", duration=30)
        stop_paint = ugv.PaintOff()

        dock_to_carrot = ugv.OmniDock(load_value="carrot")
        dock_to_pothole = ugv.OmniDock(load_value="pothole")

        root.add_children(
            [   
            wait_for_completion,
            dock_to_pothole,
            # start_paint,
            # dock_to_carrot,
            # move_forward,
            # stop_paint,
            ]
        )

        root.add_children(
            [   
            wait_for_completion,
            start_paint,
            move_forward,
            stop_paint,
            ]
        )

        return root


def main():
    rospy.init_node("paint_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = PaintBT()

    rospy.spin()


if __name__ == "__main__":
    main()
