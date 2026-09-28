# UAV Exploration and 3D Reconstruction

Real time planners that explore an unknown space with a UAV and reconstruct it as they go.
Four planners share one pipeline, two classic and two kinodynamic.

| planner | what it is |
|---|---|
| AEP | Autonomous Exploration Planner |
| RH-NBVP | Receding Horizon Next Best View Planner |
| KAEP | Kinodynamic AEP |
| KRH-NBVP | Kinodynamic RH-NBVP |

The kinodynamic planners build a Kinodynamic RRT and pick viewpoints that maximise expected
information gain against flight cost, respecting the UAV model and its constraints.

AEP and RH-NBVP can also score viewpoints by marginal gain, which counts only the space a
viewpoint sees beyond what the path leading to it already covers, so overlapping views along a
branch are not counted twice. Gain is evaluated on the GPU, which is what makes this path
dependent formulation affordable online.

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
The Gazebo worlds come with the `uav_gazebo_environments` submodule, fetched with the same
protocol as the main clone: an SSH clone gets it over SSH, an HTTPS clone over HTTPS.
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
If you clone without ```--recursive```, fetch the submodule afterwards:
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
cd ~/catkin_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone
./start.sh
```
### Multi-Drone Simulation

For a three-drone simulation:

```bash
cd ~/catkin_ws/src/UAV_3D_reconstruction/multi/multi_motion_planning/tmux/three_drones
./start.sh
```
To configure which simulation scenario and algorithms to run, edit the ```session.yml``` file accordingly. This follows the standard MRS UAV System format. 

You can find additional MRS examples in the [mrs_core_examples](https://github.com/ctu-mrs/mrs_core_examples) repository.

# Environments

The Gazebo worlds live in their own repository,
[uav_gazebo_environments](https://github.com/joaom2a0r0i1a/uav_gazebo_environments),
linked here as the `uav_gazebo_environments` submodule and cloned with the repository (see
[Clone the repository](#2-clone-the-repository)). The submodule is pinned to the commit this
code was tested with. To move it to the newest `main`:

```bash
git submodule update --remote uav_gazebo_environments
```

Do not keep a second clone of it elsewhere in the workspace, catkin refuses two
`uav_gazebo_environments` packages.

It carries six worlds and, for each, the three regions the pipeline needs, where the planner
may sample, where gain is counted and what the evaluation measures.

Switching environment is one name. Set `environment` in the planner config, or export
`PLANNER_ENV`, and every node picks up the right regions:

```yaml
environment: warehouse
```

Choosing a planner is the same idea:

```bash
roslaunch motion_planning planner.launch planner:=kaep      # or rhnbvp, aep, krhnbvp
```

# Running Experiments

An experiment is one flight in the single-drone simulation. The flight is recorded and scored
afterwards. The scoring needs `python3-scipy` and `python3-matplotlib`.

### 1. Fly

The settings of an experiment are at the top of
`single/motion_planning/tmux/one_drone/session.yml`:

```yaml
  - export PLANNER_KIND=${PLANNER_KIND:-rhnbvp}                                   # aep | rhnbvp | kaep | krhnbvp
  - export PLANNER_ENV=${PLANNER_ENV:-school}                                     # school | police | warehouse | multistory | big_maze | maze
  - export EXP_TIME_LIMIT=${EXP_TIME_LIMIT:-30}                                   # [min] flight length
  - export EXP_DATA_DIR=${EXP_DATA_DIR:-$(rospack find motion_planning)/data}    # run and bag folder
```

Change the value after `:-`. Give every condition its own data folder, for example
`$(rospack find motion_planning)/data/aep_school`, so the conditions can be compared later. To
choose the gain of AEP and RH-NBVP, add `marginal_gain:=true` or `marginal_gain:=false` to the
`planner.launch` line further down the same file. Then start the simulation:

```bash
cd ~/catkin_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone
./start.sh
```

The drone takes off, the planner starts on its own and the flight ends at the time limit. The run
is saved in the data folder as `<date>_<time>/`, with the saved maps, `voxblox_data.csv` and
`data_log.txt`, and its recording goes to `tmp_bags/` next to it. Close the session with
`./kill.sh` and start again for the next run.

### 2. Score each run

```bash
DATA=$(rospack find motion_planning)/data
roslaunch motion_planning full_voxblox_eval.launch target_directory:=$DATA/aep_school \
    environment:=school method:=all evaluate_volume:=true create_meshes:=true error_histogram:=true
```

This scores every run in `aep_school`.

- `environment` selects the region that is scored, the same one the drone flew in.
- School and police are compared with their ground truth cloud,
  `uav_gazebo_environments/ground_truth/<environment>.ply`, which gives the mean error, RMSE and
  unknown voxels of every saved map. `evaluate_volume:=true` adds the reconstructed volume.
- The other worlds have no ground truth cloud. `evaluate:=false` turns the comparison off, so they
  are scored by reconstructed volume alone, which is why `evaluate_volume:=true` must stay on.
- `create_meshes` and `error_histogram` add a mesh of every saved map and a histogram of the errors.

The scores are added to each run's `voxblox_data.csv` and the figures go to its `graphs/`. A run
is only scored once, running the command again skips it.

### 3. Compare conditions

```bash
roslaunch motion_planning full_voxblox_eval.launch target_directory:=$DATA environment:=school \
    multi_series:=true series_labels:=aep_school,rhnbvp_school | tee ~/series.log
```

Use the same `evaluate` settings as in step 2. The conditions are plotted together in
`$DATA/multi_series_evaluation/`, and for each one the time to reach 25, 50, 75 and 95 % coverage
and the final coverage are printed. Coverage is the known part of the ground truth for school and
police, and the reconstructed volume over the volume of the region for the other worlds.

### 4. Paper metrics

```bash
export MP=$(rospack find motion_planning)
S=$MP/scripts/evaluation/analysis
python3 $S/milestones_from_log.py ~/series.log   # E25 to E95 and final coverage
python3 $S/path_vel_mapped.py aep_school         # path length and average velocity
python3 $S/termination_time.py aep_school        # AEP termination time
```

The last two read the condition from `$MP/data`. The gain accuracy and computation time come from
the gain benchmark, described in
[scripts/evaluation/figures](single/motion_planning/scripts/evaluation/figures/README.md).

# Notes
- For reproducibility of the results shown in the papers below, ensure you are using the specified versions of **MRS** and the **customized Voxblox** repository linked above.
- Performance may vary depending on your hardware (slower hardware may lead to worse results). The experiments of the marginal gain on the GPU paper were conducted using:
  - **CPU:** Intel® Core™ i7-13650HX (13th Gen)
  - **GPU:** NVIDIA GeForce RTX 5060 Laptop GPU

# Credits

If you use this work, please cite the paper that corresponds to the part you use.

**Kinodynamic planning**, published in IEEE Robotics and Automation Letters.

```bibtex
@article{Mendes_2026,
  author  = {Mendes, Jo{\~a}o F{\'e}lix and Basiri, Meysam and Ventura, Rodrigo},
  title   = {Kinodynamic Trajectory Planning for Efficient UAV Exploration
             and Reconstruction of Unknown Environments},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {2},
  pages   = {1530--1537},
  doi     = {10.1109/LRA.2025.3641147}
}
```

**Centralized multi UAV exploration**, accepted and presented at ICARM 2026, IEEE Xplore entry
pending. Cite as to appear until the DOI exists.

Joao Felix Mendes, Meysam Basiri and Rodrigo Ventura, "Centralized Multi-UAV Exploration and
3D Reconstruction Using Single-UAV Planners", ICARM 2026.

**Marginal gain on the GPU**, submitted to ICRA and under review.
