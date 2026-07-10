#!/bin/bash
#
# study1-2_eval.sh — two-stage voxblox eval for Study 1 (AEP RRT) and Study 2
# (AEP RRT*), run inside the noetic_ws container. Stage 1 per label adds error
# columns + SimulationOverview.png; stage 2 compares each study's two labels
# (scoped via series_labels) and the multi_series output is archived per study.

set -u
CTR=noetic_ws
ONE=/home/ros1/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone
WS=/home/ros1/ros1_motion_ws
DATA_CTR="$WS/src/UAV_3D_reconstruction/motion_planning/data"

RENV="source /opt/ros/noetic/setup.bash && source $WS/devel/setup.bash && export MPLBACKEND=Agg"

stage1() {  # $1 = label
  echo "=== stage1: $1 ==="
  docker exec "$CTR" bash -lc "$RENV && roslaunch motion_planning full_voxblox_eval.launch \
    target_directory:=$DATA_CTR/$1 method:=all"
}

stage2() {  # $1 = studyname  $2 = "LABEL_A,LABEL_B"
  echo "=== stage2 compare: $1  ($2) ==="
  docker exec "$CTR" bash -lc "$RENV && roslaunch motion_planning full_voxblox_eval.launch \
    multi_series:=true series_labels:=\"$2\""
  # archive so the next study's compare doesn't overwrite it
  docker exec "$CTR" bash -lc "rm -rf $DATA_CTR/multi_series_$1 && \
    cp -r $DATA_CTR/multi_series_evaluation $DATA_CTR/multi_series_$1 && \
    echo archived -> data/multi_series_$1"
}

echo "############ Study 1 — AEP GPU abs vs marg RRT ############"
stage1 GPU_abs_RRT
stage1 GPU_marg_RRT
stage2 study1 "GPU_abs_RRT,GPU_marg_RRT"

echo "############ Study 2 — AEP GPU abs vs marg RRT* ############"
stage1 GPU_abs_RRTstar
stage1 GPU_marg_RRTstar
stage2 study2 "GPU_abs_RRTstar,GPU_marg_RRTstar"

echo "############ EVAL DONE ############"
