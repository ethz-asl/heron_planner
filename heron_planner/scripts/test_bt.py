#!/usr/bin/env python

import rospy

import py_trees as pt
import ros_trees as rt

import heron_planner.trees.base_bt as base_bt

import heron_planner.leaves.ugv_behaviours as ugv
import heron_planner.leaves.hlp_behaviours as hlp
import heron_planner.leaves.iccs_behaviours as iccs
import heron_planner.leaves.generic_behaviours as generic


class TestBT(base_bt.BaseBT):
    def __init__(self) -> None:
        super().__init__("TestBT")

    def load_parameters(self) -> None:

        self.test_img = rospy.get_param("/test/img")
        self.test_arm = rospy.get_param("/test/arm")
        self.test_comp = rospy.get_param("/test/comp")
        self.tree_rate = rospy.get_param("tree_rate", 10)
        self.inspection_names = rospy.get_param("pothole/inspection_names")
        self.body_cam_ns = rospy.get_param(
            "ugv/body_cam_ns", "/robot/base_camera/front_rgbd_camera/"
        )
        self.arm_cam_ns = rospy.get_param(
            "ugv/arm_cam_ns", "/robot/arm_camera/front_rgbd_camera/"
        )

    def save_to_blackboard(self) -> None:
        self.bb.set("arm_cam_ns", self.arm_cam_ns)
        self.bb.set("body_cam_ns", self.body_cam_ns)

    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="InspectionSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()
        load_arm_img_inspection = hlp.GetSynchedImages(
            task_name="Load wrist image",
            load_key="arm_cam_ns",
            image_key="cone/rgb",
            save=True,
        )

        send_inspection_to_kafka = hlp.SendImageToKafka(
            task_name="Send inspection to Kafka",
            msg="Real image for HLP testing",
            load_key="cone/rgb",
        )

        roller_down = ugv.RollerDown()
        roller_up = ugv.RollerUp()
        roller_cmd_half = ugv.RollerCommand(
            task_name="roller half way", load_value=0.5
        )
        blow = ugv.Blow()
        blow_pothole = ugv.BlowPothole()

        take_snap = ugv.TakeSnap()
        move_arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to home", load_value="home"
        )
        move_arm_to_inspection_mid = ugv.MoveArmTo(
            task_name="Move arm to inspection_mid",
            load_value="inspection_mid",
        )
        move_arm_to_inspection_left = ugv.MoveArmTo(
            task_name="Move arm to inspection_left",
            load_value="inspection_left",
        )
        move_arm_to_inspection_right = ugv.MoveArmTo(
            task_name="Move arm to inspection_right",
            load_value="inspection_right",
        )

        if self.test_img:
            take_photo_seq = pt.composites.Sequence(
                name="PhotoSequence",
                children=[load_arm_img_inspection, send_inspection_to_kafka],
                memory=True,
            )
            root.add_children([take_photo_seq])


        if self.test_comp:
            roller_test_seq = pt.composites.Sequence(
                name="ComponentTestSequence", children=[roller_down, roller_up]
            )
            root.add_children([roller_test_seq, blow])

        if self.test_arm:
            move_arm_seq = pt.composites.Sequence(
                name="MoveArmSequence",
                children=[
                    move_arm_to_home,
                    move_arm_to_inspection_mid,
                    take_snap,
                    move_arm_to_inspection_left,
                    take_snap,
                    move_arm_to_inspection_right,
                    take_snap,
                ],
            )
            root.add_children([move_arm_seq])

        return root


def main():
    rospy.init_node("test_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = TestBT()
    node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
