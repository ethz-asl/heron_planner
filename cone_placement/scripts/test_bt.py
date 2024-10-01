#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

from ..src.utils import transform_utils as utils
from geometry_msgs.msg import Pose

def calculate_sweep(start: Pose, pothole: Pose, diameter: float):
    """
    pose_a : start
    pose_b : goal

    finds 3 straight trajectories all parallel to pose_a -> pose_b
    length of trajectories : dist_2 (default is dist_1 * 2)
    
    (P1 -> P2) goes through (pose_a -> pose_b) with pose_b as midpoint
    (P3 -> P4)  & (P5 -> P6) are parallel to (P1 -> P2) with dist_1 / 2

    returns 
    """

    sweep_1 = utils.find_parallel_pose(pothole, start, diameter, towards=True)
    sweep_2 = utils.find_perpendicular_pose(sweep_1, pothole, diameter/2.0, towards=True)
    sweep_3 = utils.find_perpendicular_pose(sweep_1, pothole, diameter/2.0, towards=False)

    return sweep_1, sweep_2, sweep_3

# Visualize the points
def plot_points(pose_a, pose_b, P1, P2, P3):
    """Plots the calculated points."""
    fig, ax = plt.subplots()

    point_a = utils.point_from_pose(pose_a)
    point_b = utils.point_from_pose(pose_b)

    # Plot the original poses
    ax.plot(point_a[0], point_a[1], 'kx', label='Pose A (Start)')
    ax.plot(point_b[0], point_b[1], 'kx', label='Pose B (Pothole)')

    point_1 = utils.point_from_pose(P1)
    point_2 = utils.point_from_pose(P2)
    point_3 = utils.point_from_pose(P3)

    # Plot P1, P3, P5
    ax.plot(point_1[0], point_1[1], color='lightgrey', marker='o', label='P1')
    ax.plot(point_2[0], point_2[1], color='lightgrey', marker='o', label='P2')
    ax.plot(point_3[0], point_3[1], color='lightgrey', marker='o', label='P3')

    # Draw lines to show the connections
    ax.plot([point_a[0], point_b[0]], [point_a[1], point_b[1]], color='lightgrey', linestyle='--', label="Pothole <-> Start")
    ax.plot([point_2[0], point_1[0]], [point_2[1], point_1[1]], color='lightgrey', linestyle='--', label="P2 Perpendicular")
    ax.plot([point_3[0], point_1[0]], [point_3[1], point_1[1]], color='lightgrey', linestyle='--', label="P3 Perpendicular")

    # Formatting
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    ax.grid(True)
    plt.title('Visualization of P1, P3, and P5')
    plt.show()

pose_a = Pose()
pose_a.position.x = 10
pose_a.position.y = -5
pose_a.orientation.z = 1

pose_b = Pose()
pose_b.position.x = 5
pose_b.position.y= 3
pose_b.orientation.z = 1

# Calculate points
dist_1 = 2.0  # Diameter of pothole

P1, P3, P5 = calculate_sweep(pose_a, pose_b, dist_1)

# Plot the results
plot_points(pose_a, pose_b, P1, P3, P5)