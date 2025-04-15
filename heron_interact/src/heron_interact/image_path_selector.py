import os
import rospy
import rospkg
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QComboBox, QPushButton
from python_qt_binding.QtCore import Qt, QPoint

from PyQt5.QtGui import QImage, QPixmap

MAX_POINTS = 10

class ImagePathSelector(Plugin):
    def __init__(self, context):
        super(ImagePathSelector, self).__init__(context)
        self.setObjectName("ImagePathSelector")

        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("heron_interact"),
            "resource",
            "PathSelector.ui",
        )
        loadUi(ui_file, self._widget)

        self._widget.setObjectName("ImagePathSelectorUi")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        self.bridge = CvBridge()
        self.subscriber = None
        self.latest_image = None
        self.image_topic = ""

        self.points = []
        self.selection_active = False

        # Connect UI
        self._widget.comboBox_topic.currentTextChanged.connect(
            self.on_topic_selected
        )
        self._widget.button_start.clicked.connect(self.start_selection)
        self._widget.button_undo.clicked.connect(self.undo_point)
        self._widget.button_clear.clicked.connect(self.clear_points)
        self._widget.button_send.clicked.connect(self.send_path)
        self._widget.label_image.mousePressEvent = self.image_clicked

        self.populate_topic_list()

    def populate_topic_list(self):
        # Get list of image topics (this can be improved later)
        import rostopic

        topics = rospy.get_published_topics()
        image_topics = [t[0] for t in topics if t[1] == "sensor_msgs/Image"]
        self._widget.comboBox_topic.addItems(image_topics)

    def on_topic_selected(self, topic):
        if self.subscriber:
            self.subscriber.unregister()
        self.image_topic = topic
        self.subscriber = rospy.Subscriber(topic, Image, self.image_callback)

    def image_callback(self, msg):
        try:
            cv_img = self.convert_ros_image(msg)
            if cv_img is not None:
                self.latest_image = cv_img
                self.update_image()
            else:
                rospy.logerr(f"Image conversion failed.")
        except CvBridgeError as e:
            rospy.logerr(e)

    def convert_ros_image(self, msg):
        try:
            # directly convert to rgb
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv_img
        except CvBridgeError as err:
            try:
                # fallback to raw conversion
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                if msg.encoding == "16UC1" or msg.encoding == "32FC1":
                    img_arr = np.array(cv_img, dtype=np.float32)

                    # dynamic range stretch
                    min_val = np.min(img_arr)
                    max_val = np.max(img_arr)
                    if max_val == min_val:
                        max_val = min_val + 1

                    img_scaled = ((img_arr - min_val) / (max_val - min_val) * 255).astype(np.uint8)
                    img_rgb = cv2.cvtColor(img_scaled, cv2.COLOR_GRAY2RGB)
                    return img_rgb
            
                elif msg.encoding in ["mono8", "8UC1"]:
                    return cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)
                
                elif msg.encoding == "8UC3":
                    return cv_img  # assume it's already RGB/BGR
                
                else:
                    rospy.logwarn(f"Unsupported image encoding: {msg.encoding}")
                    return None
            except CvBridgeError as err:
                rospy.logerr(f"Conversion failed: {err}")
                return None

    def update_image(self):
        if self.latest_image is None:
            return
        img = self.latest_image.copy()
        for pt in self.points:
            # cv2.circle(img, pt, 5, (0, 0, 255), -1)
            cv2.drawMarker(
                img, 
                pt, 
                color=[204, 0, 102], 
                markerType=cv2.MARKER_TILTED_CROSS,
                line_type=cv2.LINE_AA,
                markerSize=30,
                thickness=3
            )
        height, width, channel = img.shape
        bytes_per_line = 3 * width
        qt_img = QImage(
            img.data, width, height, bytes_per_line, QImage.Format_RGB888
        ).rgbSwapped()
        self._widget.label_image.setPixmap(QPixmap.fromImage(qt_img))

    def image_clicked(self, event):
        if not self.selection_active or self.latest_image is None:
            return
        if len(self.points) >= MAX_POINTS:
            rospy.logwarn(f"Reached max points!")
            return
        label = self._widget.label_image
        x = int(event.pos().x() * self.latest_image.shape[1] / label.width())
        y = int(event.pos().y() * self.latest_image.shape[0] / label.height())
        self.points.append((x, y))
        self.update_image()

    def start_selection(self):
        self.selection_active = True

    def undo_point(self):
        if self.points:
            self.points.pop()
            self.update_image()

    def clear_points(self):
        self.points = []
        self.update_image()

    def send_path(self):
        rospy.loginfo("FAKE: Send path clicked (placeholder)")
