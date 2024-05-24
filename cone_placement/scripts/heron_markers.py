from __future__ import annotations
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

from utils import marker_utils

DELETE_MARKER_MSG = Marker(action=Marker.DELETEALL)
DELETE_MARKER_ARRAY_MSG = MarkerArray(markers=[DELETE_MARKER_MSG])


class DrawCones:
    def __init__(self):
        self.cone_markers = []
        self.pubs = None
        self._create_publishers()

    def _create_publishers(self):
        self.pubs = dict()
        self.pubs["cones"] = rospy.Publisher(
            "cones", MarkerArray, queue_size=1, latch=True
        )

    def draw_cones(
        self,
        poses: list,
        colors: np.array,
        ids: np.array,
        frame: str = "map",
        height: float = 0.5,
        radius: float = 0.05,
    ):
        """
        draw list of cones, cylinder marker with text over the top
        """

        for _, (pose, color, id) in enumerate(zip(poses, colors, ids)):
            self.add_cone(pose, id, frame, color, height=height, radius=radius)

        msg = MarkerArray(markers=self.cone_markers)
        self.pubs["cones"].publish(msg)

    def draw_cone(
        self,
        pose: Pose,
        id: int,
        color: str | list,
        frame: str = "map",
        height: float = 0.5,
        radius: float = 0.05,
    ):
        """
        draw singular cone
        """
        self.add_cone(pose, id, frame, color, height=height, radius=radius)
        msg = MarkerArray(markers=self.cone_markers)
        self.pubs["cones"].publish(msg)

    def add_cone(
        self,
        pose: Pose,
        id: int,
        frame: str = "map",
        cone_color: str | list = "orange",
        text_color: str | list = "white",
        height: float = 0.5,
        radius: float = 0.05,
    ):
        """
        get marker msg, add cone to it
        """
        text_buffer = 0.05
        cone_msg = marker_utils.create_cylinder_marker_msg(
            pose, cone_color, frame, height, radius
        )
        cone_msg.id = int(id * 2)
        pose.position.z = pose.position.z + (height / 2.0) + text_buffer
        text = "cone_" + str(id)
        text_msg = marker_utils.create_text_marker_msg(
            pose, text, text_color, frame
        )
        text_msg.id = int((id * 2) + 1)

        self.cone_markers.append(cone_msg)
        self.cone_markers.append(text_msg)

    """
    id = 0, cyl.id = 0 text.id = 1
    id = 1, cyl.id = 2 text.id = 3
    id = 2, cyl.id = 4, text.id = 5
    id = 3, cyl.id = 6, text.id = 6
    """

    def clear_cone(self, id: int):
        """
        from id, find cone and text and set marker to delete
        """
        cone_id = int(id * 2)
        text_id = int(cone_id + 1)

        self.cone_markers[cone_id] = marker_utils.delete_marker_msg(
            self.cone_markers[cone_id]
        )
        self.cone_markers[text_id] = marker_utils.delete_marker_msg(
            self.cone_markers[text_id]
        )

        msg = MarkerArray(markers=self.cone_markers)
        self.pubs["cones"].publish(msg)

    def show_cone(self, id: int):
        """
        from id, find cone and text and set marker to delete
        """
        cone_id = int(id * 2)
        text_id = int(cone_id + 1)

        self.cone_markers[cone_id] = marker_utils.show_marker_msg(
            self.cone_markers[cone_id]
        )
        self.cone_markers[text_id] = marker_utils.show_marker_msg(
            self.cone_markers[text_id]
        )

        msg = MarkerArray(markers=self.cone_markers)
        self.pubs["cones"].publish(msg)

    def clear(self):
        self.clear_cones()

    def clear_cones(self):
        self.cone_markers = []
        self.pubs["cones"].publish(DELETE_MARKER_ARRAY_MSG)
