#!/usr/bin/env python

import rospy

import py_trees as pt
import ros_trees as rt

import heron_planner.trees.base_bt as base_bt

import heron_planner.leaves.ugv_behaviours as ugv
import heron_planner.leaves.hlp_behaviours as hlp
import heron_planner.leaves.iccs_behaviours as iccs
import heron_planner.leaves.generic_behaviours as generic


class ConePlaceBT(base_bt.BaseBT):
    def __init__(self) -> None:
        super().__init__("ConePlaceBT")

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

    def get_kafka_photo_seq(
        self,
        img_key: str,
        kafka_msg: str = "",
        cam_ns: str = "arm_cam_ns",
        seq_task_name="KafkaImageSeq",
        load_img_task_name="Get images",
        send_kafka_task_name="Send image to kafka",
    ) -> pt.composites.Composite:
        """"""
        load_img = hlp.GetSynchedImages(
            task_name=load_img_task_name,
            load_key=cam_ns,
            image_key=img_key,
            save=True,
        )

        send_img_to_kafka = hlp.SendImageToKafka(
            task_name=send_kafka_task_name,
            msg=kafka_msg,
            load_key=img_key,
        )

        return pt.composites.Sequence(
            name=seq_task_name,
            children=[load_img, send_img_to_kafka],
            memory=True,
        )

    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="ConePlaceSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to home",
            load_value="home",
        )

        move_forward = ugv.Move(
            task_name="move forward", load_value="MOVE 3.0 0"
        )

        move_diagonal = ugv.Move(
            task_name="move diagonal", load_value="MOVE 1.5 1.5"
        )

        pick_up_cone1 = ugv.PickUpFrom(
            task_name="Pick up cone 1 from robot", load_value="robot"
        )
        place_cone1 = ugv.PlaceOn(
            task_name="Place cone 1 on floor", load_value="floor"
        )


        pick_up_cone2 = ugv.PickUpFrom(
            task_name="Pick up cone 2 from robot", load_value="robot"
        )
        place_cone2 = ugv.PlaceOn(
            task_name="Place cone 2 on floor", load_value="floor"
        )

        pick_up_cone3 = ugv.PickUpFrom(
            task_name="Pick up cone 3 from robot", load_value="robot"
        )
        place_cone3 = ugv.PlaceOn(
            task_name="Place cone 3 on floor", load_value="floor"
        )

        pick_up_cone4 = ugv.PickUpFrom(
            task_name="Pick up cone 4 from robot", load_value="robot"
        )
        place_cone4 = ugv.PlaceOn(
            task_name="Place cone 4 on floor", load_value="floor"
        )

        root.add_children(
            [
                arm_to_home,
                pick_up_cone1,
                place_cone1,
                move_forward,
                pick_up_cone2,
                place_cone2,
                # move_forward,
                # pick_up_cone3,
                # place_cone3,
                # move_diagonal,
                # pick_up_cone4,
                # place_cone4
            ]
        )

        return root


def main():
    rospy.init_node("cone_place_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = ConePlaceBT()
    # node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
