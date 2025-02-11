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

```bash


```bash
./install_dependencies.sh
```

## Installation with docker (WIP)
