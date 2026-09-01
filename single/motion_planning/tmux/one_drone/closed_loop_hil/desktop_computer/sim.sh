#!/bin/bash
# Raw-tmux launcher for the DESKTOP HIL Path B session (no tmuxinator).
# Same socket/session name as the normal sim (mrs / simulation), so your usual
# kill works:  tmux -L mrs kill-session -t simulation
# Sim + control + sensors only — voxblox/planner/cache run on the Orin.
SDIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
[ -z "$DESKTOP_IP" ] && [ -f "$SDIR/../config.sh" ] && source "$SDIR/../config.sh"   # standalone: read IP from config.sh (campaign passes it via env)
: "${DESKTOP_IP:?DESKTOP_IP not set — copy config.sh.template to config.sh, or export DESKTOP_IP}"
cd "$SDIR/../.."   # closed_loop_hil/desktop_computer/ -> tmux/one_drone/ (holds config/ + current_config.env)
S=simulation ; L=mrs
PRE="export ROS_MASTER_URI=http://${DESKTOP_IP}:11311; export ROS_IP=${DESKTOP_IP}; export UAV_NAME=uav1; export RUN_TYPE=simulation; export UAV_TYPE=f450; export WORLD_NAME=simulation; export PX4_SIM_SPEED_FACTOR=1.0; [ -f ./current_config.env ] && source ./current_config.env"
T="tmux -L $L"
# $1 = "window" or "window.pane"; $2 = command (single-quote it in the caller so $VARs/$(...) stay literal
# and get expanded by the pane's own shell after PRE runs)
send() { $T send-keys -t "$S:$1" "$PRE; $2" Enter; }

$T kill-session -t $S 2>/dev/null

$T -f /etc/ctu-mrs/tmux.conf new-session -d -s $S -n roscore
send roscore 'roscore'

$T new-window -t $S -n gazebo
send gazebo 'waitForRos; roslaunch mrs_uav_gazebo_simulation simulation.launch world_file:=$(rospack find environments)/worlds/grass_plane_school.world gui:=true'
$T split-window -t $S:gazebo
send gazebo.1 'waitForGazebo; rosservice call /mrs_drone_spawner/spawn "1 --$UAV_TYPE --pos -22 0 0.5 0 --enable-rangefinder --enable-ground-truth --enable-realsense-front-pitched"'

$T new-window -t $S -n status  ; send status  'waitForHw; roslaunch mrs_uav_status status.launch'
$T new-window -t $S -n hw_api  ; send hw_api  'waitForTime; roslaunch mrs_uav_px4_api api.launch custom_config:=./config/hw_api.yaml'
$T new-window -t $S -n core    ; send core    'waitForHw; roslaunch mrs_uav_core core.launch platform_config:=`rospack find mrs_uav_gazebo_simulation`/config/mrs_uav_system/$UAV_TYPE.yaml custom_config:=./config/custom_config.yaml world_config:=./config/world_config.yaml network_config:=./config/network_config.yaml'
$T new-window -t $S -n takeoff ; send takeoff 'waitForHw; roslaunch mrs_uav_autostart automatic_start.launch custom_config:=./config/automatic_start.yaml'
$T split-window -t $S:takeoff  ; send takeoff.1 'waitForControl; rosservice call /$UAV_NAME/hw_api/arming 1; sleep 2; rosservice call /$UAV_NAME/hw_api/offboard'
$T new-window -t $S -n pointclouds ; send pointclouds 'waitForControl; roslaunch motion_planning cam_to_ptcld.launch'
$T split-window -t $S:pointclouds  ; send pointclouds.1 'waitForControl; roslaunch motion_planning process_pointcloud.launch config_pcl_filter_rs_front_pitched:=./config/rs_front_pitched_filter.yaml'
$T new-window -t $S -n rviz    ; send rviz    'waitForControl; rosrun rviz rviz -d ./config/custom_rviz.rviz'

echo ">>> desktop sim launched (socket '$L', session '$S')."
echo ">>> attach:  tmux -L $L attach -t $S"
echo ">>> kill:    tmux -L $L kill-session -t $S"
