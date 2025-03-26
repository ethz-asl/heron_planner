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


class VisionBT(base_bt.BaseBT):
    def __init__(self) -> None:
        super().__init__("VisionTestBT")

    def load_parameters(self) -> None:
        self.tree_rate = rospy.get_param("tree_rate", 10)
        self.inspection_names = rospy.get_param("/pothole/inspection_names")
        self.body_cam_ns = rospy.get_param(
            "/ugv/body_cam_ns", "/robot/body_camera"
        )
        self.arm_cam_ns = rospy.get_param("/ugv/arm_cam_ns", "/robot/arm_camera")
        self.arm_cam_tf = rospy.get_param("/ugv/arm_cam_tf", "")
        self.use_kafka = rospy.get_param("/kafka", False)

    def save_to_blackboard(self) -> None:
        self.bb.set("arm_cam_ns", self.arm_cam_ns)
        self.bb.set("body_cam_ns", self.body_cam_ns)
        self.bb.set("inspections", self.inspection_names)

        fake_dock_pose = PoseStamped()
        fake_dock_pose.header.frame_id = "odom"
        fake_dock_pose.header.stamp = rospy.Time.now()
        fake_dock_pose.pose.position.x = 0.0
        fake_dock_pose.pose.position.y = 0.0
        fake_dock_pose.pose.position.z = 0.0
        fake_dock_pose.pose.orientation.w = 1.0

        self.bb.set("fake_dock_pose", fake_dock_pose)

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
        load_img_task_name="Get images for kafka",
        send_kafka_task_name="Send image to kafka",
    ) -> pt.composites.Composite:
        """"""
        load_img = hlp.GetSynchedImages(
            task_name=load_img_task_name,
            load_key=cam_ns,
            image_key=img_key,
            save=True,
        )

        if self.use_kafka:
            send_img_to_kafka = hlp.SendImageToKafka(
                task_name=send_kafka_task_name, msg=kafka_msg, load_key=img_key
            )
        else:
            send_img_to_kafka = hlp.FakeSendImageToKafka(
                task_name=send_kafka_task_name, msg=kafka_msg, load_key=img_key
            )

        return pt.composites.Sequence(
            name=seq_task_name,
            children=[load_img, send_img_to_kafka],
            memory=True,
        )

    def find_pothole_seq(
        self,
        img_key: str,
        cam_ns: str = "arm_cam_ns",
        seq_task_name: str = "FindPotholeSeq",
        load_img_task_name: str = "Get img from arm",
        find_task_name: str = "Find pothole",
    ) -> pt.composites.Composite:

        load_img = hlp.GetSynchedImages(
            task_name=load_img_task_name,
            load_key=cam_ns,
            image_key=img_key,
            save=True,
        )
        find_pothole = iccs.FindPothole(task_name=find_task_name)
        transform_pothole = hlp.TransformPose(task_name="Transform pothole to odom",
                                              target_frame="odom",
                                              load_key="/pothole/com",
                                              save_key="/pothole/odom")
        find_offset = hlp.FindOffset(
            defect="pothole",
            broadcast=True,
            broadcast_frame="pothole_offset",
            load_key="/pothole/odom",
        )

        return pt.composites.Sequence(
            name=seq_task_name,
            children=[load_img, find_pothole, transform_pothole, find_offset],
            memory=True,
        )

    def find_crack_seq(
        self,
        img_key: str,
        cam_ns: str = "arm_cam_ns",
        seq_task_name: str = "FindCrackSeq",
        load_img_task_name: str = "Get img from arm",
        find_task_name: str = "Find crack",
    ) -> pt.composites.Composite:

        load_img = hlp.GetSynchedImages(
            task_name=load_img_task_name,
            load_key=cam_ns,
            image_key=img_key,
            save=True,
        )
        find_crack = iccs.FindCrack(task_name=find_task_name)
        generate_crack_path = hlp.GenerateCrackPath()
        find_offset = hlp.FindOffset(
            defect="crack",
            broadcast=True,
            broadcast_frame="crack_offset",
            load_key="/crack/middle"
        )

        # TODO add here a confirmation
        # TODO get center of path & docking pose
        # TODO move to crack dock
        # TODO relook at path
        # TODO add path interpolator here
        # TODO move to "pose"

        return pt.composites.Sequence(
            name=seq_task_name,
            children=[load_img, find_crack, generate_crack_path, find_offset],
            memory=True,
        )

    def get_inspection_loop(self) -> pt.composites.Composite:
        """loop through inspection positons and find pothole"""

        inspection_mid = self.move_take_snap(
            move_loc="inspection_mid_old",
            seq_task_name="MoveToInspectionMidSeq",
        )
        mid_photo = self.get_kafka_photo_seq(
            img_key="/pothole/mid", kafka_msg="pothole/inspection-mid"
        )

        inspection_left = self.move_take_snap(
            move_loc="inspection_left", seq_task_name="MoveToInspectionLeftSeq"
        )
        left_photo = self.get_kafka_photo_seq(
            img_key="/pothole/left", kafka_msg="pothole/inspection-left"
        )

        inspection_right = self.move_take_snap(
            move_loc="inspection_right",
            seq_task_name="MoveToInspectionRightSeq",
        )
        right_photo = self.get_kafka_photo_seq(
            img_key="/pothole/right", kafka_msg="pothole/inspection-right"
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

    def build_root(self) -> pt.behaviour.Behaviour:
        """build root"""

        root = pt.composites.Sequence(name="VisionTestSequence", memory=True)

        wait_for_enter = generic.WaitForEnterKey()

        arm_to_home = ugv.MoveArmTo(
            task_name="Move arm to home", load_value="home"
        )
        arm_to_inspection_mid = ugv.MoveArmTo(
            task_name="Move arm to inspection mid",
            load_value="inspection_mid_old",
        )
        pothole_photo = self.find_pothole_seq(img_key="/pothole/inspection")
        crack_photo = self.find_crack_seq(img_key="/crack/inspection")

        dock_midpoint = ugv.OmniDock(load_value="fake_dock")
        move_to_pose = ugv.MoveToPose()
        # then we would want to dock to position
        # then inspection left
        # redo crack photo

        take_snap = ugv.TakeSnap()

        reverse_to_inspection = ugv.Move(
            task_name="reverse 1.3m", load_value="MOVE -1.3 0.0"
        )

        find_pothole = iccs.FindPothole(task_name="Find pothole")

        root.add_children(
            [   
                arm_to_home,
                self.get_inspection_loop(),
                crack_photo
                # pothole_photo
            ]
        )

        return root


def main():
    rospy.init_node("vision_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = VisionBT()
    # node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
