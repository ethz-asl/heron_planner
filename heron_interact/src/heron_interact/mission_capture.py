#!/usr/bin/env python

import rospy
import rospkg
import glob
import json
import os

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

# FROM KAFKA -> heron.remvis.mission
#TODO convert this to ui panel


def load_defect_coords(data_dir: str) -> list:

    files = sorted(glob.glob(os.path.join(data_dir, "defect-*.json")))
    coords = []

    for fpath in files:
        with open(fpath, 'r') as file:
            data = json.load(file)
            centroid = data["bounding_box"]["centroid"]
            coords.append((centroid["latitude"], centroid["longitude"], data["type"].lower()))

    return coords

def load_mission_coords(data_dir: str) -> list:

    files = sorted(glob.glob(os.path.join(data_dir, "mission-*.json")))
    coords = []

    for fpath in files:
        with open(fpath, 'r') as file:
            data = json.load(file)
            for pt in data["polygon"]:
                coords.append((pt["latitude"], pt["longitude"], data["missionID"]))

    return coords

def create_pubs(defect_coords: list, mission_coords: list) -> list:
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

def pub_navsat(pubs: list, combined_coords: list): 

    rate = rospy.Rate(1) 
    seq = 0

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for idx, pub in enumerate(pubs):
            lat = combined_coords[idx][0]
            lon = combined_coords[idx][1]
            msg = NavSatFix(
                header=Header(
                    stamp=now, frame_id=f"defect_gps_{idx}", seq=seq
                ),
                latitude = lat,
                longitude = lon,
                altitude = 0.0, # no altitude
                position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )
            pub.publish(msg)

        seq += 1
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("mission_capture")

    folder = rospkg.RosPack().get_path("heron_interact") + "/config"
    defect_coords = load_defect_coords(folder)
    mission_coords = load_mission_coords(folder)
    pubs = create_pubs(defect_coords, mission_coords)

    combined_coords = defect_coords + mission_coords

    rospy.loginfo(f"Publishing {len(pubs)} defects")
    pub_navsat(pubs, combined_coords)
