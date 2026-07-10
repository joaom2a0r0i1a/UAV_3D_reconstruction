#!/bin/bash
#
# study3_eval.sh — two-stage voxblox eval for Study 3 (RH-NBVP abs vs marg RRT),
# run inside noetic_ws. Mirrors study1-2_eval.sh.

set -u
CTR=noetic_ws
WS=/home/ros1/ros1_motion_ws
DATA_CTR="$WS/src/UAV_3D_reconstruction/motion_planning/data"
RENV="source /opt/ros/noetic/setup.bash && source $WS/devel/setup.bash && export MPLBACKEND=Agg"

stage1() {
  echo "=== stage1: $1 ==="
  docker exec "$CTR" bash -lc "$RENV && roslaunch motion_planning full_voxblox_eval.launch \
    target_directory:=$DATA_CTR/$1 method:=all"
}

echo "############ Study 3 — RH-NBVP GPU abs vs marg RRT ############"
stage1 NBV_abs_RRT
stage1 NBV_marg_RRT

echo "=== stage2 compare: study3 (NBV_abs_RRT,NBV_marg_RRT) ==="
docker exec "$CTR" bash -lc "$RENV && roslaunch motion_planning full_voxblox_eval.launch \
  multi_series:=true series_labels:=\"NBV_abs_RRT,NBV_marg_RRT\" 2>&1 | grep -i 'Timing corresponding'"
docker exec "$CTR" bash -lc "rm -rf $DATA_CTR/multi_series_study3 && \
  cp -r $DATA_CTR/multi_series_evaluation $DATA_CTR/multi_series_study3 && \
  echo archived -> data/multi_series_study3"

echo "############ EVAL STUDY3 DONE ############"
