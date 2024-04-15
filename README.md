# Heron Planner
Repository for the high-level planner for the automated [HERON](https://www.heron-h2020.eu/) project - which aims to develop an integrated system for the maintenance and repairs on roadworks. 

## (not so) Quick start

> Note that all the instructions in this repository are tested on Ubuntu 20.04 wit ROS noetic.

First create a catkin workspace, and a `src` folder, and clone the repository.

```bash
mkdir -p ~/heron_ws/src
cd ~/heron_ws/src

git clone git@github.com:ethz-asl/heron_planner.git
```

This repository is complimentary to the [moma](https://github.com/ethz-asl/moma) repository, which should be cloned into the `src` folder of the workspace alongside its submodules. 

```bash
git clone --recurse-submodules git@github.com:ethz-asl/moma.git
```

Install the general system dependencies of the `moma` repository (requires sudo rights).
```bash
./moma/install_dependencies.sh
```

Checkout the `feature/panda-on-gmp` branch on the `moma` repository. Unless [this](https://github.com/ethz-asl/moma/pull/173) pull request has been approved, then you can stay on `master`. 

If using the **giraffe** configuration (this is the base [GMP](https://github.com/ethz-asl/robolab_gmp/) and the Franka Panda arm), clone the GMP repository with its submodules.
```bash
git clone --recurse-submodules git@github.com:ethz-asl/robolab_gmp.git
```
> **note** the `robolab_gmp` repo is a whole catkin workspace and requires later refactor. 

#### Building with Catkin
Go to the project folder.

```bash
cd ~/heron_ws
```

Set catkin configuration for faster execution.
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Then build **all project packages**.

```bash
catkin build
```

> **note** some of the packages in `robolab_gmp` produce some warnings, that can be ignored.

Then source the workspace!
```bash
cd ~/heron_ws
source devel/setup.bash
```

### Preliminary demo
To see the giraffe in action against many cone hazards, run:
```bash
roslaunch heron_demo run_gazebo.launch
```

This should load RViz and Gazebo. In RViz, you can move the base using the pink 2D Nav Goal. The panda arm can be moved with the interactive markers or through the motion planning interface.

>Note, the project sources depend on several ROS packages. If errors occur related to missing packages during compilation and building, **install** the corresponding **prerequisites** via `sudo apt install <missing package>` or via the corresponding github repository.
