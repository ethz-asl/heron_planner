from __future__ import annotations
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

from utils import marker_utils

DELETE_MARKER_MSG = Marker(action=Marker.DELETEALL)
DELETE_MARKER_ARRAY_MSG = MarkerArray(markers=[DELETE_MARKER_MSG])

class DrawCylinders:
    def __init__(self, pub_name: str = "cylinders"):
        self._pub_name = pub_name
        self.markers = []
        self.pubs = None
        self._create_publishers()

    def _create_publishers(self):
        self.pubs = dict()
        self.pubs[self._pub_name] = rospy.Publisher(
            self._pub_name, MarkerArray, queue_size=1, latch=True
        )

    def draw_markers(
        self,
        poses: list,
        colors: np.array,
        ids: np.array,
        frame: str = "map",
        height: float = 0.5,
        radius: float = 0.05,
    ):
        """
        draw list of cylinder markers with text over the top
        """

        for _, (pose, color, id) in enumerate(zip(poses, colors, ids)):
            self.add_cylinder(pose, id, frame, color, height=height, radius=radius)

        msg = MarkerArray(markers=self.markers)
        self.pubs[self._pub_name].publish(msg)

    def draw_cylinder(
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
        self.add_cylinder(pose, id, frame, color, height=height, radius=radius)
        msg = MarkerArray(markers=self.markers)
        self.pubs["cones"].publish(msg)

    def add_cylinder(
        self,
        pose: Pose,
        id: int,
        frame: str = "map",
        cylinder_color: str | list = "orange",
        text_color: str | list = "white",
        height: float = 0.5,
        radius: float = 0.05,
    ):
        """
        get marker msg, add cylinder and text to it
        """
        text_buffer = 0.05
        cylinder_msg = marker_utils.create_cylinder_marker_msg(
            pose, cylinder_color, frame, height, radius
        )
        cylinder_msg.id = int(id * 2)
        pose.position.z = pose.position.z + (height / 2.0) + text_buffer
        text = self._pub_name + "_" + str(id)
        text_msg = marker_utils.create_text_marker_msg(
            pose, text, text_color, frame
        )
        text_msg.id = int((id * 2) + 1)

        self.markers.append(cylinder_msg)
        self.markers.append(text_msg)

    """
    id = 0, cyl.id = 0 text.id = 1
    id = 1, cyl.id = 2 text.id = 3
    id = 2, cyl.id = 4, text.id = 5
    id = 3, cyl.id = 6, text.id = 6
    """

    def clear_marker(self, id: int):
        """
        from id, find cylinder and text and set marker to delete
        """
        cylinder_id = int(id * 2)
        text_id = int(cylinder_id + 1)

        self.markers[cylinder_id] = marker_utils.delete_marker_msg(
            self.markers[cylinder_id]
        )
        self.markers[text_id] = marker_utils.delete_marker_msg(
            self.markers[text_id]
        )

        msg = MarkerArray(markers=self.markers)
        self.pubs[self._pub_name].publish(msg)

    def show_marker(self, id: int):
        """
        from id, find cylinder and text and set marker to show
        """
        cylinder_id = int(id * 2)
        text_id = int(cylinder_id + 1)

        self.markers[cylinder_id] = marker_utils.show_marker_msg(
            self.markers[cylinder_id]
        )
        self.markers[text_id] = marker_utils.show_marker_msg(
            self.markers[text_id]
        )

        msg = MarkerArray(markers=self.markers)
        self.pubs[self._pub_name].publish(msg)

    def clear(self):
        self.clear_markers()

    def clear_markers(self):
        self.markers = []
        self.pubs[self._pub_name].publish(DELETE_MARKER_ARRAY_MSG)


class DrawText:
    def __init__(self, topic_name: str):
        self.text_markers = []
        self.pubs = None
        self.state = rospy.get_param(topic_name, "start")
        self._create_publishers()

    def _create_publisher(self):
        self.pubs = dict()
        self.pubs["text"] = rospy.Publisher(
            "text_marker", MarkerArray, queue_size=1, latch=True
        )

    def draw_text(
        self,
        text: str,
        pose: Pose,
        id: int,
        frame: str = "map",
    ):
        """
        draw text
        """
        self.add_text(text, pose, id, frame)
        msg = MarkerArray(markers=self.text_markers)
        self.pubs["text"].publish(msg)

    def add_text(
        self,
        text: str,
        pose: Pose,
        id: int,
        frame: str = "map",
        text_color: str | list = "white",
    ):
        """
        get marker msg
        """
        text_msg = marker_utils.create_text_marker_msg(
            pose, text, text_color, frame
        )
        text_msg.id = id

        self.text_markers.append(text_msg)

    """
    id = 0, cyl.id = 0 text.id = 1
    id = 1, cyl.id = 2 text.id = 3
    id = 2, cyl.id = 4, text.id = 5
    id = 3, cyl.id = 6, text.id = 6
    """

    def clear_text(self, id: int):
        """
        from id, set marker to delete
        """
        self.text_markers[id] = marker_utils.delete_marker_msg(
            self.text_markers[id]
        )

        msg = MarkerArray(markers=self.text_markers)
        self.pubs["text"].publish(msg)

    def show_text(self, id: int):
        """
        from id, find cone and text and set marker to delete
        """
        self.text_markers[id] = marker_utils.show_marker_msg(
            self.text_markers[id]
        )

        msg = MarkerArray(markers=self.text_markers)
        self.pubs["text"].publish(msg)

    def clear(self):
        self.clear_text()

    def clear_text(self):
        self.text_markers = []
        self.pubs["text"].publish(DELETE_MARKER_ARRAY_MSG)
