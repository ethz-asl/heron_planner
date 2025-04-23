#!/usr/bin/env python

import json
import cv2
import os
import glob
import numpy as np
from pyproj import Transformer
import rospy

from sensor_msgs.msg import NavSatFix

# ========== config ==========
DATA_DIR = "config"
CANVAS_SIZE = 800

# ========== find and load the JSON ==========

mission_file = glob.glob(os.path.join(DATA_DIR, "mission-*.json"))[0]
defect_files = glob.glob(os.path.join(DATA_DIR, "defect-*.json"))

with open(mission_file, "r") as file:
    mission = json.load(file)

defects = []
for fpath in defect_files:
    with open(fpath, "r") as file:
        defects.append(json.load(file))


# ========== setup up projection ==========

# utm zone 34N for Athens
transformer = Transformer.from_crs("EPSG:4326", "EPSG:32634", always_xy=True)

def to_xy(lat, lon):
    x, y = transformer.transform(lon, lat)
    return x, y

# ========== convert gps -> utm ==========

mission_pts = [to_xy(pt["latitude"], pt["longitude"]) for pt in mission["polygon"]]
defect_pts = [to_xy(defect["bounding_box"]["centroid"]["latitude"],
                        defect["bounding_box"]["centroid"]["longitude"]) for defect in defects]


# combine for bounds
all_x = [x for x, _ in mission_pts + defect_pts]
all_y = [y for _, y in mission_pts + defect_pts]

x_min, x_max = np.min(all_x), np.max(all_x)
y_min, y_max = np.min(all_y), np.max(all_y)

# ========== publish to NavSat ==========



# ========== convert to img pixels ==========

def to_pixel_coords(x, y):
    px = int((x - x_min) / (x_max - x_min) * CANVAS_SIZE)
    py = int((y_max - y) / (y_max - y_min) * CANVAS_SIZE) # y axis flip

    return px, py

mission_px = [to_pixel_coords(x, y) for x, y in mission_pts]
defect_px = [to_pixel_coords(x, y) for x, y in defect_pts]

print(f"mission_px : {mission_px}")
print(f"defect_px : {defect_px}")

# ========== draw on img ==========

img = np.ones((CANVAS_SIZE, CANVAS_SIZE, 3), dtype=np.uint8) * 255

# draw mission cones
for cone_px in mission_px:
    cv2.circle(img, cone_px, radius=5, color=(0, 154, 255), thickness=-1)

# draw defects 
for px in defect_px:
    cv2.circle(img, px, radius=10, color=(188, 255, 0), thickness=-1)

cv2.imshow("Projected map", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
