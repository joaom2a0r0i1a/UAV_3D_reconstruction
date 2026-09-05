#!/bin/bash
# Profiled flight recorder. See README.md for which profile to use when.
# Usage: record.sh [profile]   profile = eval | eval-viz | eval-camera | mapping-replay | full-debug

PROFILE="${1:-eval}"
UAV="${UAV_NAME:-uav1}"
EXP_DIR="${EXP_DIR:-$HOME/real_experiments/current}"
BIG_DIR="$HOME/bag_files/$(date +%Y_%m_%d)"
HEARTBEAT="${RECORD_HEARTBEAT:-15}"

# Scored runs need these. path_vel reads the pose, eval_real shifts the box by offset_out.
EVAL_TOPICS=(
  /mavros/local_position/pose
  /mavros/local_position/velocity_local
  /mavros/state
  /mavros/setpoint_raw/local
  /tf
  /tf_static
  /$UAV/offset_out
  /$UAV/simulation_ready
)

# Published by voxblox_node (processed_voxblox.launch) and by the planner itself.
VIZ_TOPICS=(
  /$UAV/visualization_marker_out
  /$UAV/frustum_out
  /$UAV/unknown_voxels_out
  /$UAV/voxblox_node/mesh
  /$UAV/voxblox_node/occupied_nodes
  # Occupancy clouds, free and occupied per voxel. Heavy, near 1 GB per 10 min each.
  /$UAV/voxblox_node/esdf_pointcloud
  /$UAV/voxblox_node/tsdf_pointcloud
  /$UAV/voxblox_node/surface_pointcloud
)

CAMERA_TOPICS=(
  /camera/color/image_raw/compressed
  /camera/color/camera_info
  # Raw aligned depth: with rgb + camera_info + tf this is a complete rtabmap / Open3D input,
  # and it also allows re-running voxblox offline finer than the 0.2 m the planner needs live.
  /camera/aligned_depth_to_color/image_raw
)

# Enough to re-run cam_to_ptcld + voxblox offline. Colour is stored compressed, so a replay
# needs:  rosrun image_transport republish compressed in:=/camera/color/image_raw raw out:=/camera/color/image_raw
MAPPING_TOPICS=(
  /camera/aligned_depth_to_color/image_raw
  /camera/color/camera_info
  /camera/color/image_raw/compressed
  /tf
  /tf_static
  /mavros/local_position/pose
  /mavros/state
)

# Runs rosbag in the background so the window keeps reporting that recording is alive.
run_record() {
  local out="$1"; shift
  mkdir -p "$(dirname "$out")"
  echo "[record] profile   : $PROFILE"
  echo "[record] label     : ${RUN_LABEL:-unset}   uav: $UAV"
  echo "[record] output    : ${out}_<date>.bag"
  if [ "$#" -gt 0 ]; then
    echo "[record] topics    : $#"
    printf '[record]            %s\n' "$@"
  else
    echo "[record] topics    : all, minus the exclude regex"
  fi
  echo "[record] ---- starting, Ctrl+C stops and closes the bag ----"

  rosbag record -o "$out" "${RECORD_ARGS[@]}" "$@" &
  local rb=$!
  # rosbag record is a python wrapper around the C++ recorder that owns the bag, so signal the
  # child too: SIGINT to the wrapper alone leaves the bag unindexed as a .active file.
  stop_rosbag() {
    local c
    for c in $(ps -o pid= --ppid "$rb" 2>/dev/null); do kill -INT "$c" 2>/dev/null; done
    kill -INT "$rb" 2>/dev/null
  }
  trap 'echo "[record] signal received, closing the bag"; stop_rosbag' INT TERM

  local t0=$SECONDS f sz
  while kill -0 "$rb" 2>/dev/null; do
    sleep "$HEARTBEAT"
    kill -0 "$rb" 2>/dev/null || break
    f=$(ls -t "${out}"*.bag.active 2>/dev/null | head -1)
    sz=$([ -n "$f" ] && du -h "$f" 2>/dev/null | cut -f1 || echo "-")
    printf '[record] RECORDING  %5ds  %7s  %s\n' "$((SECONDS-t0))" "$sz" "$(basename "${f:-waiting}")"
  done

  wait "$rb"; local rc=$?
  f=$(ls -t "${out}"*.bag 2>/dev/null | head -1)
  if ls "${out}"*.bag.active > /dev/null 2>&1; then
    echo "[record] STOPPED after $((SECONDS-t0))s but a .active file remains, the bag was NOT closed:"
    ls -la "${out}"*.bag.active
    echo "[record] recover it with: rosbag reindex <file>.bag.active && mv <file>.bag.active <file>.bag"
  else
    echo "[record] STOPPED after $((SECONDS-t0))s, bag closed: ${f:-none} ($([ -n "$f" ] && du -h "$f" | cut -f1))"
  fi
  return $rc
}

RECORD_ARGS=()
case "$PROFILE" in
  eval)
    # __name must stay eval_bag_recorder; eval_data_node_real.py kills that node to close the bag.
    RECORD_ARGS=(__name:=eval_bag_recorder)
    run_record "$EXP_DIR/tmp_bags/tmp_bag" "${EVAL_TOPICS[@]}"
    ;;
  eval-viz)
    # same destination and node name as eval, so the run still scores, just with more topics
    RECORD_ARGS=(__name:=eval_bag_recorder)
    run_record "$EXP_DIR/tmp_bags/tmp_bag" "${EVAL_TOPICS[@]}" "${VIZ_TOPICS[@]}"
    ;;
  eval-camera)
    RECORD_ARGS=(__name:=eval_bag_recorder)
    run_record "$EXP_DIR/tmp_bags/tmp_bag" "${EVAL_TOPICS[@]}" "${VIZ_TOPICS[@]}" "${CAMERA_TOPICS[@]}"
    ;;
  mapping-replay)
    run_record "$BIG_DIR/mapping" "${MAPPING_TOPICS[@]}"
    ;;
  full-debug)
    RECORD_ARGS=(--all --exclude "(.*)(theora|compressedDepth|parameter_descriptions|parameter_updates|/bond)(.*)|/camera/color/image_raw/compressed/(.*)|/camera/aligned_depth_to_color/image_raw/compressed(.*)")
    run_record "$BIG_DIR/full"
    ;;
  *)
    echo "unknown profile '$PROFILE' (eval|eval-viz|eval-camera|mapping-replay|full-debug)"; exit 1
    ;;
esac
