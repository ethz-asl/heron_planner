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
        self.cone_offset = rospy.get_param("/cone_place/offset", 0.7)

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

    def go_to_cone(self, cone_id : str = "cone_1", seq_task_name="ConePlaceSeq"):
        # go to cone location
        go_to_cone = ugv.GoTo(task_name=f"Go to {cone_id}")
    
        # move sideways 0.7
        move_to_offset = ugv.Move(
            task_name="move to offset", load_value=f"MOVE 0.0 {self.cone_offset}"
        )

        # place cone
        pick_up_cone = ugv.PickUpFrom(
            task_name="Pick up cone 1 from robot", load_value="robot"
        )
        place_cone = ugv.PlaceOn(
            task_name="Place {cone 1} on floor", load_value="floor"
        )

        return pt.composites.Sequence(
            name=seq_task_name,
            children=[go_to_cone, move_to_offset, pick_up_cone, place_cone],
            memory=True
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
        #TODO automated cone

        root = pt.composites.Sequence(name="ConePlaceSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to home",
            load_value="home",
        )


        dock_to_cone1 = ugv.OmniDock(
            task_name="Dock to cone1", load_value="cone_1"
        )
        dock_to_cone2 = ugv.OmniDock(
            task_name="dock to cone2", load_value="cone_2"
        )

        go_to_cone1 = ugv.GoTo(
            task_name="Go to cone 1", load_key="cone_1"
        )
        go_to_cone2 = ugv.GoTo(
            task_name="Go to cone 1", load_key="cone_1"
        )
        go_to_cone3 = ugv.GoTo(
            task_name="Go to cone 1", load_key="cone_1"
        )

        move_forward = ugv.Move(
            task_name="move forward", load_value="MOVE 5.0 0"
        )
        turn = ugv.Turn(
            task_name=""
        )

        move_diagonal = ugv.Move(
            task_name="move diagonal", load_value="MOVE 2.5 2.5"
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
                dock_to_cone1,
                pick_up_cone1,
                place_cone1,
                dock_to_cone2,
                pick_up_cone2,
                place_cone2,
                # pick_up_cone3,
                # place_cone3,
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
