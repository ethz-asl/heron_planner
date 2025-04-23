import os
import glob
import json
import subprocess
import rospy
import rospkg
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, TriggerRequest
from cv_bridge import CvBridge, CvBridgeError

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QComboBox, QPushButton
from python_qt_binding.QtCore import Qt, QPoint

from PyQt5.QtGui import QImage, QPixmap

MAX_CLICKS = rospy.get_param("/cracks/path_length", 10)
PATH_TOPIC = rospy.get_param("/cracks/path_topic", "/hlp/path")
BODY_CAM_NS = rospy.get_param("/ugv/body_cam_ns", "/robot/body_camera")
ARM_CAM_NS = rospy.get_param("/ugv/arm_cam_ns", "/robot/arm_camera")
ARM_CAM_FRAME = rospy.get_param("/ugv/arm_cam_frame", "front_rgbd_camera_rgb_camera_optical_frame")
BODY_CAM_FRAME = rospy.get_param("/ugv/body_cam_frame", "front_rgbd_camera_rgb_camera_optical_frame")
PATH_FRAME = rospy.get_param("/ugv/base_frame", "robot_base_footprint")

class HeronGUI(Plugin):
    def __init__(self, context):
        super(HeronGUI, self).__init__(context)
        self.setObjectName("HeronGUI")

        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("heron_interact"),
            "resource",
            "Heron.ui",
        )
        loadUi(ui_file, self._widget)

        self._widget.setObjectName("HeronUi")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        self.bridge = CvBridge()
        self.subscriber = None
        self.latest_image = None

        self.depth_img = None
        self.cam_info = None

        self.img_sub = None
        self.depth_sub = None
        self.info_sub = None

        self.bt_process = None

        self.img_topic = ""
        self.depth_topic = ""
        self.info_topic = ""

        self.pixels = []

        self.path_pub = rospy.Publisher(PATH_TOPIC, Path, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Connect UI
        # panel 1 - mission
        self._widget.button_load_mission.clicked.connect(self.load_mission)
        self._widget.combo_bt_type.setCurrentIndex(0)
        self.bt_selected(self._widget.combo_bt_type.currentText())
        self._widget.combo_bt_type.currentTextChanged.connect(self.bt_selected)

        self._widget.button_start.clicked.connect(self.start_bt)
        self._widget.button_stop.clicked.connect(self.stop_bt)
        self._widget.button_pause.clicked.connect(self.pause_bt)


        # panel 2 - img selector
        self._widget.comboBox_ns.addItems(["Arm Camera", "Body Camera"])
        self._widget.comboBox_ns.setCurrentIndex(0) # set default selection
        self.cam_selected(self._widget.comboBox_ns.currentText())
        self._widget.comboBox_ns.currentTextChanged.connect(
            self.cam_selected
        )
        self._widget.button_undo.clicked.connect(self.undo_point)
        self._widget.button_clear.clicked.connect(self.clear_points)
        self._widget.button_send.clicked.connect(self.send_path)
        self._widget.label_image.mousePressEvent = self.image_clicked

        rospy.Subscriber("/hlp/state", String, self.hlp_state_cb)

    def hlp_state_cb(self, msg: String):
        self._widget.label_status.setText(f"Status: {msg.data}")

    def load_mission(self):

        print("PRESSED LOAD MISSION")
        folder = rospkg.RosPack().get_path("heron_interact") + "/config"
        defect_coords = self.load_defect_coords(folder)
        mission_coords = self.load_mission_coords(folder)
        pubs = self.create_pubs(defect_coords, mission_coords)

        combined_coords = defect_coords + mission_coords

        rospy.loginfo(f"Publishing {len(defect_coords)} defects")
        self._widget.label_defects.setText(f"No. defects found: {str(len(defect_coords))}")
        self.pub_navsat(pubs, combined_coords)


    def load_defect_coords(self, data_dir: str) -> list:

        files = sorted(glob.glob(os.path.join(data_dir, "defect-*.json")))
        coords = []

        for fpath in files:
            with open(fpath, 'r') as file:
                data = json.load(file)
                centroid = data["bounding_box"]["centroid"]
                coords.append((centroid["latitude"], centroid["longitude"], data["type"].lower()))

        return coords

    def load_mission_coords(self, data_dir: str) -> list:

        files = sorted(glob.glob(os.path.join(data_dir, "mission-*.json")))
        coords = []

        for fpath in files:
            with open(fpath, 'r') as file:
                data = json.load(file)
                for pt in data["polygon"]:
                    coords.append((pt["latitude"], pt["longitude"], data["missionID"]))

        return coords

    def create_pubs(self, defect_coords: list, mission_coords: list) -> list:
        pubs = []
        for idx, data in enumerate(defect_coords):
            _, _, defect_type = data
            topic = f"/hlp/{defect_type}_gps_{idx}"
            pub = rospy.Publisher(topic, NavSatFix, queue_size=1)
            pubs.append(pub)

        for idx, data in enumerate(mission_coords):
            topic = f"/hlp/cone_gps_{idx}"
            pub = rospy.Publisher(topic, NavSatFix, queue_size=1)
            pubs.append(pub)

        return pubs

    
    def pub_navsat(self, pubs: list, combined_coords: list): 

        now = rospy.Time.now()
        for idx, pub in enumerate(pubs):
            lat = combined_coords[idx][0]
            lon = combined_coords[idx][1]
            msg = NavSatFix(
                header=Header(
                    stamp=now, frame_id=f"defect_gps_{idx}"
                ),
                latitude = lat,
                longitude = lon,
                altitude = 0.0, # no altitude
                position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )
            pub.publish(msg)

    def bt_selected(self, bt_type):
        rospy.loginfo(f"[RQT] BT selected: {bt_type}")

        # self.stop_bt()  # always stop any existing BT node

        # map BT types to scripts
        script_map = {
            "Cracks": "crack_bt.py",
            "Potholes": "pothole_bt.py",
            "Cone Placement": "cone_place_bt.py"
        }

        if bt_type not in script_map:
            rospy.logwarn(f"[RQT] Unknown BT type selected: {bt_type}")
            return

        script_name = script_map[bt_type]
        pkg_name = "heron_planner"

        try:
            # use rospack to find the path to the package
            pkg_path = subprocess.check_output(["rospack", "find", pkg_name]).decode().strip()
            script_path = os.path.join(pkg_path, "scripts", script_name)

            if not os.path.exists(script_path):
                rospy.logerr(f"[RQT] BT script not found: {script_path}")
                return

            # Launch it as a subprocess
            self.bt_process = subprocess.Popen(["rosrun", pkg_name, script_name])
            rospy.loginfo(f"[RQT] Launched BT: {script_name}")

        except Exception as e:
            rospy.logerr(f"[RQT] Failed to start BT: {e}")

    def start_bt(self):
        try: 
            rospy.wait_for_service("/hlp/start", timeout=3)
            srv = rospy.ServiceProxy("/hlp/start", Trigger)
            req = TriggerRequest()
            srv(req)
        except rospy.ServiceException as err:
            rospy.logerr(f"Srv call failed: {err}")
        except rospy.ROSException as ros_err:
            rospy.logerr(f"Service /hlp/start not available: {ros_err}")

    def pause_bt(self):
        try: 
            rospy.wait_for_service("/hlp/pause", timeout=3)
            srv = rospy.ServiceProxy("/hlp/pause", Trigger)
            req = TriggerRequest()
            srv(req)
        except rospy.ServiceException as err:
            rospy.logerr(f"Srv call failed: {err}")
        except rospy.ROSException as ros_err:
            rospy.logerr(f"Service /hlp/pause not available: {ros_err}")

    def stop_bt(self):
        try: 
            rospy.wait_for_service("/hlp/stop", timeout=3)
            srv = rospy.ServiceProxy("/hlp/stop", Trigger)
            req = TriggerRequest()
            srv(req)
        except rospy.ServiceException as err:
            rospy.logerr(f"Srv call failed: {err}")
        except rospy.ROSException as ros_err:
            rospy.logerr(f"Service /hlp/stop not available: {ros_err}")

        if self.bt_process is not None:
            if self.bt_process.poll() is None: # still running
                self.bt_process.terminate()
                try:
                    self.bt_process.wait(timeout=3)
                    rospy.loginfo("[RQT] BT stopped successfully")
                except subprocess.TimeoutExpired:
                    rospy.logwarn("[RQT] Force filling BT...")
                    self.bt_process.kill()
            self.bt_process = None


    def cam_selected(self, cam_type):

        if cam_type == "Arm Camera":
            rospy.loginfo("Using arm camera")
            ns = ARM_CAM_NS
            self.cam_frame = ARM_CAM_FRAME
        elif cam_type == "Body Camera":
            rospy.loginfo("Using body camera")
            ns = BODY_CAM_NS
            self.cam_frame = BODY_CAM_FRAME
        
        self.img_topic = ns + "/rgb/image_raw"
        self.depth_topic = ns + "/stereo/image_raw"
        # self.info_topic = ns + "/stereo/camera_info"
        self.info_topic = ns + "/rgb/camera_info" # TODO check which one is valid

        if self.img_sub:
            self.img_sub.unregister()

        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb)
        
        if self.depth_sub:
            self.depth_sub.unregister()

        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_cb)

        if self.info_sub:
            self.info_sub.unregister()

        self.info_sub = rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb)

    def depth_cb(self, msg):
        self.depth_img = msg


    def info_cb(self, msg):
        self.cam_info = msg

    def img_cb(self, msg):
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
        for pixel in self.pixels:
            # cv2.circle(img, pt, 5, (0, 0, 255), -1)
            cv2.drawMarker(
                img, 
                pixel, 
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
        if self.latest_image is None:
            return
        if len(self.pixels) >= MAX_CLICKS:
            rospy.logwarn(f"Reached max points!")
            return
        label = self._widget.label_image
        x = int(event.pos().x() * self.latest_image.shape[1] / label.width())
        y = int(event.pos().y() * self.latest_image.shape[0] / label.height())
        self.pixels.append((x, y))
        self.update_image()

    def undo_point(self):
        if self.pixels:
            self.pixels.pop()
            self.update_image()

    def clear_points(self):
        self.pixels = []
        self.update_image()

    def project_pixels_to_points(self, pixels):
        
        if self.depth_img is None or self.cam_info is None:
            return None

        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"Depth image conversion error: {e}")
            return None


        K = np.array(self.cam_info.K).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1] # focal lengths
        cx, cy = K[0, 2], K[1, 2] # principal point
        
        points = []
        for (u, v) in pixels:
            z = depth_img[v, u]/1000.0 # depth at pixel
            rospy.loginfo(f"Pixel ({u}, {v}) depth = {z}")
            if z > 0:
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                
                points.append((x,y,z))
                # convert to optical frame
                # opt_x = z
                # opt_y = -x
                # opt_z = -y

                # points.append((opt_x, opt_y, opt_z))
            else:
                rospy.logerr(f"Error with depth")

        return points


    def transform_pose(self, pose: PoseStamped, target_frame: str = "odom") -> PoseStamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # Target frame (e.g., "odom" or "robot_map")
                pose.header.frame_id,  # Source frame ("camera_link")
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            return tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(f"TF Transform error: {err}")
            return None

    def send_path(self):
        # get pixel positions to 3d here
        # need to add tick box area
        if not self.pixels:
            rospy.logerr("Select some points first!")
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = PATH_FRAME
        # path_msg.header.frame_id = self.cam_frame

        # scale = 0.001 # in meters
        scale = 1 # in meters
        points = self.project_pixels_to_points(self.pixels)
        for (x, y, z) in points:
            pose = PoseStamped()
            pose.header.frame_id = self.cam_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x * scale
            pose.pose.position.y = y * scale
            pose.pose.position.z = z * scale
            pose.pose.orientation.w = 1 # assume points up

            # tranform pose to robot frame
            transformed_pose = self.transform_pose(pose, PATH_FRAME)
            if transformed_pose:
                transformed_pose.pose.position.z = 0 # road assumed 2D
                transformed_pose.pose.orientation.w = 1 # fixed orientation so obselete
                transformed_pose.header.stamp = path_msg.header.stamp
                path_msg.poses.append(transformed_pose)
            else:
                rospy.logwarn(f"Failed to transform path to {PATH_FRAME}")

            # path_msg.poses.append(pose)


        self.path_pub.publish(path_msg)
        rospy.loginfo(f"published path with [{len(path_msg.poses)}] points")
        

