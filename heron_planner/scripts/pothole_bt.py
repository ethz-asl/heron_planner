#!/usr/bin/env python

import rospy

import py_trees as pt
import ros_trees as rt

import heron_planner.trees.base_bt as base_bt

import heron_planner.leaves.ugv_behaviours as ugv
import heron_planner.leaves.hlp_behaviours as hlp
import heron_planner.leaves.iccs_behaviours as iccs
import heron_planner.leaves.generic_behaviours as generic


class PotholeBT(base_bt.BaseBT):
    def __init__(self) -> None:
        super().__init__("PotholeTestBT")

    def load_parameters(self) -> None:
        self.tree_rate = rospy.get_param("tree_rate", 10)
        self.inspection_names = rospy.get_param("pothole/inspection_names")
        self.body_cam_ns = rospy.get_param(
            "ugv/body_cam_ns", "/body_camera/body_camera"
        )
        self.arm_cam_ns = rospy.get_param(
            "ugv/arm_cam_ns", "/arm_camera/arm_camera"
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

    def get_inspection_loop(self) -> pt.composites.Composite:
        """loop through inspection positons and find pothole"""

        inspection_mid = self.move_take_snap(
            move_loc="inspection_mid_old", seq_task_name="MoveToInspectionMidSeq"
        )
        mid_photo = self.get_kafka_photo_seq(
            img_key="/pothole/mid", kafka_msg="inspection mid pothole"
        )

        inspection_left = self.move_take_snap(
            move_loc="inspection_left", seq_task_name="MoveToInspectionLeftSeq"
        )
        left_photo = self.get_kafka_photo_seq(
            img_key="/pothole/left", kafka_msg="inspection left pothole"
        )

        inspection_right = self.move_take_snap(
            move_loc="inspection_right",
            seq_task_name="MoveToInspectionRightSeq",
        )
        right_photo = self.get_kafka_photo_seq(
            img_key="/pothole/right", kafka_msg="inspection right pothole"
        )

        return pt.composites.Sequence(
            name="inspectionLoop",
            children=[
                inspection_mid,
                mid_photo,
                inspection_left,
                left_photo,
                inspection_right,
                right_photo,
            ],
        )

    def get_pothole_sel(self) -> pt.composites.Composite:
        find_pothole_mid = iccs.FindPothole(
            task_name="Find pothole at inspection mid", load_key="pothole/mid"
        )
        find_pothole_left = iccs.FindPothole(
            task_name="Find pothole at inspection left", load_key="pothole/left"
        )
        find_pothole_right = iccs.FindPothole(
            task_name="Find pothole at inspection right",
            load_key="pothole/right",
        )

        return pt.composites.Sequence(
            name="FindPotholeSel",
            children=[find_pothole_mid, find_pothole_left, find_pothole_right],
            memory=False,
        )

    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="PotholeSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to home",
            load_value="home",
        )
        arm_to_inspection_mid = ugv.MoveArmTo(
            task_name="Move arm to inspection mid",
            load_value="inspection_mid_old",
        )
        take_snap = ugv.TakeSnap()

        reverse_1 = ugv.Move(task_name="reverse 1m", load_value="MOVE -1.0 0.0")
        reverse_half = ugv.Move(task_name="reverse 0.5m", load_value="MOVE -0.5 0.0")

        forward_1 = ugv.Move(task_name="forward 1m", load_value="MOVE 1.0 0.0")
        forward_half = ugv.Move(task_name="forward 0.5m", load_value="MOVE 0.5 0.0")
        
        forward_to_blow = ugv.Move(task_name="forward to blow", load_value="MOVE 0.65 0")
        reverse_to_deposit = ugv.Move(task_name="reverse to deposit", load_value="MOVE -0.10 0")
        
        # dock = ugv.Dock(
        #     task_name="dock to start of offset",

        # ) # can we dock to specific frame? is this a cmd_manager cmd?

        # distance 0.25
        blow_pothole = ugv.BlowPothole()

        deposit1 = ugv.Deposit(task_name="Open deposit 1", load_value=1)        
        # deposit2 = ugv.Deposit(task_name="Open deposit 2", load_value=2)        
        # deposit3 = ugv.Deposit(task_name="Open deposit 3", load_value=3)        

        roller_up = ugv.RollerUp()
        roller_down = ugv.RollerDown()
        roller_sequence = ugv.RollerSequence(
            task_name="Pothole FB 1", load_value="FB_1"
        )
        
        root.add_children(
            [
                roller_up,
                arm_to_home,
                reverse_1,
                # arm_to_inspection_mid,
                self.get_inspection_loop(),
                arm_to_home,
                forward_to_blow,
                blow_pothole,
                reverse_to_deposit,
                deposit1,
            ]
            # [arm_to_home, self.get_inspection_loop()]
            # [ugv.TakeSnap()]
        )

        return root


def main():
    rospy.init_node("pothole_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = PotholeBT()
    # node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
