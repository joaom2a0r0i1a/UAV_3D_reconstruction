# Kinodynamic Trajectory Planning For Exploration and 3D Reconstruction
This repository contains two real-time kinodynamic trajectory planners for efficient Unmanned Aerial Vehicle (UAV) exploration: the **Kinodynamic Autonomous Exploration Planner (KAEP)** and the **Kinodynamic Receding-Horizon Next-Best-View Planner (KRH-NBVP)**. Both planners use a Kinodynamic Rapidly-Exploring Random Tree (KRRT) to evaluate and select the next-best viewpoints that maximize expected information gain while minimizing flight cost. The methods explicitly account for the UAV’s kinodynamic model and constraints, enabling fast, smooth, and feasible trajectories for exploration and 3D reconstruction tasks.

# Installation

## Prerequisites
This repository has been tested in linux with:
- Ubuntu 20.04
- ROS Noetic
- `catkin tools`
- `catkin_simple`

### 1. Install ROS Noetic (Desktop-Full is recommended). 

Find the instruction [here](https://wiki.ros.org/ROS/Installation).

### 2. Install MRS UAV System

Follow the full setup instructions from the official repository:  
[MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

### 3. Install [Voxblox](https://github.com/ethz-asl/voxblox)

By default, the project works with the **standard [Voxblox](https://github.com/ethz-asl/voxblox)** installation. Follow the installation steps [here](https://voxblox.readthedocs.io/en/latest/pages/Installation.html).

If you plan to use the **multi-robot version** of these planners, you must instead install a **modified version of Voxblox** that supports centralized multi-robot mapping.

To install the customized version, clone and build it from [here](https://github.com/joaom2a0r0i1a/feature-centralized_multi_robot_voxblox), following the same installation steps as in the official [Voxblox instructions](https://voxblox.readthedocs.io/en/latest/pages/Installation.html).

## Repository Installation

### 1. Setup your catkin workspace 

Based on the MRS UAV System setup guide (see ["Start developing your own package"](https://github.com/ctu-mrs/mrs_uav_system)).

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

# Extend the workspace with voxblox package
catkin config --extend ~/voxblox_catkin_ws/devel

# setup basic compilation profiles
catkin config --profile debug --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -Og' -DCMAKE_C_FLAGS='-Og'
catkin config --profile release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17'
catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17'
catkin profile set reldeb                     # set the reldeb profile as active
```

### 2. Clone the repository
```bash
cd ~/catkin_ws/src
```
Clone the repository using SSH (recommended) or HTTPS:
```bash
# Using SSH
git clone --recursive git@github.com:joaom2a0r0i1a/UAV_3D_reconstruction.git
# OR using HTTPS
git clone --recursive https://github.com/joaom2a0r0i1a/UAV_3D_reconstruction.git
```
If you clone without ```--recursive```, initialize submodules manually:
```bash
cd UAV_3D_reconstruction
git submodule update --init --recursive
```

### 3. Source the workspace
```bash
cd ~/catkin_ws
source devel/setup.bash
```

### 4. Build the workspace
```bash
cd ~/catkin_ws/src/UAV_3D_reconstruction
catkin build
```

# Start the Simulation

### Single Drone Simulation

To start the simulation with one drone:

```bash
cd ~/catkin_ws/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone
./start.sh
```
### Multi-Drone Simulation

For a three-drone simulation:

```bash
cd ~/catkin_ws/src/UAV_3D_reconstruction/multi_motion_planning/tmux/three_drones
./start.sh
```
To configure which simulation scenario and algorithms to run, edit the ```session.yml``` file accordingly. This follows the standard MRS UAV System format. 

You can find additional MRS examples in the [mrs_core_examples](https://github.com/ctu-mrs/mrs_core_examples) repository.

# Environments
The three Gazebo environments used to evaluate the exploration algorithms can be downloaded [here](https://github.com/joaom2a0r0i1a/UAV_3D_reconstruction/releases/tag/environments-v1/Environments.zip). They are provided as a ```.zip``` archive containing the ```.world``` files.

To use these environments with the MRS UAV System, extract the contents of the archive and move the ```.world``` files into the following directory:
```bash
/opt/ros/noetic/share/mrs_gazebo_common_resources/worlds/
```
This is the default location used by the MRS framework to load world files. After moving, you can select the desired world in your ```session.yml``` file via the ```world_name``` argument.

# Notes
- For reproducibility of the results shown in the paper, ensure you are using the specified versions of **MRS** and the **customized Voxblox** repository linked above.
- Performance may vary depending on your hardware (slower hardware may lead to worse results). The experiments in the paper were conducted using:
  - **CPU:** Intel® Core™ i9 (14th Gen)
  - **GPU:** NVIDIA GeForce RTX 4060

# Credits
If you use this work in your research, please cite the following paper:

Joao Felix Mendes, Meysam Basiri, and Rodrigo Ventura.,**“Kinodynamic Trajectory Planning for Efficient UAV Exploration and Reconstruction of Unknown Environments.”** in IEEE Robotics and Automation Letters (RAL), Accepted, November 2025.
```bash
@article{mendes2025kinodynamic,
  title={Kinodynamic Trajectory Planning for Efficient UAV Exploration and Reconstruction of Unknown Environments},
  author={Mendes, Joao Felix and Basiri, Meysam and Ventura, Rodrigo},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  note={Accepted, November 2025}
}
```