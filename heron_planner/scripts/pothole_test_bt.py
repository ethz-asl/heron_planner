#!/usr/bin/env python

import rospy

import py_trees as pt
import ros_trees as rt

import heron_planner.trees.base_bt as base_bt

import heron_planner.leaves.ugv_behaviours as ugv
import heron_planner.leaves.hlp_behaviours as hlp
import heron_planner.leaves.iccs_behaviours as iccs
import heron_planner.leaves.generic_behaviours as generic


class PotholeTestBT(base_bt.BaseBT):
    def __init__(self) -> None:
        super().__init__("PotholeTestBT")

    def load_parameters(self) -> None:
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
        self.bb.set("inspections", self.inspection_names)

    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="InspectionSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        #TODO HARDCODE THIS
        inspection_loop = pt.composites.Selector(
            name="Inspect and find pothole", memory=True
        )

        arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to",
            load_value="home",
        )

        get_inspection = generic.PopFromList(
            task_name="Get inspection",
            load_key="inspections",
            save_key="inspection",
        )
        arm_to_inspection = ugv.MoveArmTo(
            task_name="Move arm to inspection", load_key="inspection"
        )

        take_inspection_snap = ugv.TakeSnap(task_name="Take snap of inspection")
        load_arm_img_inspection = hlp.GetSynchedImages(
            task_name="Load wrist image",
            load_key="arm_cam_ns",
            image_key="pothole/rgb/before",
            save_bb_key="inspection",
            save=True,
        )

        count_pothole_success = generic.CountSuccesses(
            name="Find pothole at inspection count",
            child=iccs.FindPothole(
                task_name="Find pothole at inspection", save_bb_key="inspection"
            ),
            bb_key="pothole_detections",
        )
        inspect_location = pt.composites.Sequence(
            name="Inspect pothole",
            children=[
                get_inspection,
                arm_to_inspection,
                load_arm_img_inspection,
                take_inspection_snap,
                count_pothole_success,
            ],
            memory=False,
        )

        go_to_all_inspections = generic.Repeat(
            name="Go to inspections and take photos",
            child=inspect_location,
            num_success=len(self.inspection_names),
        )

        pothole_found = generic.ConditionSuccessThreshold(
            "Found pothole?", bb_key="pothole_detections", min_successes=1
        )
        # GOTO ALL INSPECTIONS

        send_inspection_to_kafka = hlp.SendImageToKafka(
            task_name="Send inspection to Kafka",
            msg="Real image for HLP testing",
            load_key="cone/rgb",
        )

        # example if kafka down!
        # send_inspection_to_kafka = generic.Wait(
        #     task_name="Send inspection to Kafka", duration=5
        # )

        root.add_children([go_to_all_inspections, pothole_found])

        return root


def main():
    rospy.init_node("pothole_test_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = PotholeTestBT()
    node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
