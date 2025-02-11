# HERON High Level Planner

High level planner (HLP) for the [HERON](https://www.heron-h2020.eu/) project. This planner interfaces with the robot (UGV). the vision components (ICCS), and a Kafka server to recieve missions, plan, send commands for road maintanence tasks.

The HLP uses behaviour trees (BTs) to sequence through the mission.


## Packages

- `heron_planner`: Main HLP packages with behaviour
- `heron_ui`: RViz panel with buttons to send service calls to HLP.
- `heron_utils`: Utilities for running tree, visulising tree, transforms.
- `dummy_interface`: Python utilities commonly used within moma projects.

## Installation

Note that all instructions in this repository are tested on Ubuntu 20.04 with ROS-noetic.

First, clone this repository into the `src` folder of a new or existing catkin workspace.

```bash
git clone git@github.com:ethz-asl/heron_planner.git
```

This repo depends on packages:
- [`heron_msgs`](https://github.com/RobotnikAutomation/heron_msgs/tree/main)
- [`ros_kafka_connector`](https://github.com/ethz-asl/ros-kafka-connector/tree/master)
- [`ros_trees`](https://github.com/qcr/ros_trees)

Plus `robotnik_navigation_msgs`, `robotnik_msgs` from Robotnik.

## Getting started

To run the service servers:
```bash
roslaunch heron_planner run_hlp_servers.launch
```

Then in a new terminal run the behaviour tree:
```bash
rosrun heron_planner cone_place_bt.py
```

You can view a flask server on the webpage: http://127.0.0.1:5050/


## Installation with docker (WIP)
