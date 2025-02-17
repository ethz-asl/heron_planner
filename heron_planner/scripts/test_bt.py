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
        # robot at known GPS
        #TODO compute yaw from GPS

        #LOOP
        #TODO send GPS & yaw to GOTO_GPS
        #TODO send PICKUP_FROM stack to robot
        #TODO send PLACE_ON floor to robot

        #Send msg to kafka 'cones placed'
        

        # arm_to_home = ugv.MoveTo(
        #     task_name="Move arm to home", load_value="HOME"
        # )

        # arm_to_inspection = ugv.MoveTo(
        #     task_name=f"Move arm inspection.", load_key="inspection"
        # )
        # inspect_seq = pt.composites.Sequence(
        #     name="InspectionSelector",
        #     children=[arm_to_home, arm_to_inspection],
        #     memory=True,
        # )

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

        # example if kafka down!
        # send_inspection_to_kafka = generic.Wait(
        #     task_name="Send inspection to Kafka", duration=5
        # )

        take_photo_seq = pt.composites.Sequence(
            name="PhotoSequence",
            children=[load_arm_img_inspection, send_inspection_to_kafka],
            memory=True,
        )

        root.add_children([take_photo_seq])

        return root

def main():
    rospy.init_node("test_bt")
    pt.logging.level = pt.logging.Level.DEBUG

    node = TestBT()
    node.tree.visualise()

    rospy.spin()


if __name__ == "__main__":
    main()
