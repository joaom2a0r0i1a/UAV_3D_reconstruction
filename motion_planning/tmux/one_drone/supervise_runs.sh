#!/bin/bash
#
# supervise_runs.sh — HOST-SIDE supervisor for run_experiments.sh.
#
# The noetic_ws container is interactive (docker run -it, PID1=bash, no restart
# policy), so it stops whenever its main shell ends — which has been killing runs
# ~mid-flight. This supervisor runs on the host (so it survives container death),
# drives ONE run at a time via run_experiments.sh, and on any mid-run drop it:
#   restarts the container -> discards the partial run -> retries that run.
# A run counts as good only when its voxblox_data.csv is flushed (>1 line), which
# happens on eval_data_node's clean self-stop at the end of the run.
#
# Usage: ./supervise_runs.sh [LABEL] [TARGET_RUNS] [SIM_TIME] [CONFIG_SPEC]
#   ./supervise_runs.sh GPU_marg_RRT 2 1850
#   ./supervise_runs.sh GPU_marg_RRT 2 1850 "GPU_marg_RRT:false:true:gpu:false:false"
#
# If CONFIG_SPEC (4th arg) is given it is injected into run_experiments.sh via
# EXP_CONFIG_SPEC, so no EXPLORE_CONFIGS hand-editing is needed. If omitted,
# run_experiments.sh must have exactly the matching config active in its array.

CONTAINER=noetic_ws
LABEL="${1:-GPU_marg_RRT}"
TARGET_RUNS="${2:-2}"
SIM_TIME="${3:-950}"
SPEC="${4:-}"
MAX_ATTEMPTS_PER_RUN=5

ONE_CTR=/home/ros1/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone
DATA_HOST=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/data
LABELDIR="$DATA_HOST/$LABEL"

csv_rows() { wc -l < "$1/voxblox_data.csv" 2>/dev/null || echo 0; }

good_runs() {
  local n=0 d
  for d in "$LABELDIR"/2*; do
    [ -d "$d" ] || continue
    [ "$(csv_rows "$d")" -gt 1 ] && n=$((n+1))
  done
  echo "$n"
}

purge_partials() {   # remove any un-flushed ($<=1 row) run dirs of this label
  local d
  for d in "$LABELDIR"/2*; do
    [ -d "$d" ] || continue
    if [ "$(csv_rows "$d")" -le 1 ]; then rm -rf "$d" && echo ">>> purged partial $(basename "$d")"; fi
  done
}

ensure_container() {
  if [ -z "$(docker ps --filter name="$CONTAINER" --format '{{.Names}}')" ]; then
    echo ">>> container down — starting $CONTAINER"
    docker start "$CONTAINER" >/dev/null 2>&1
    sleep 6
  fi
  # Clear ALL stale sim procs so a fresh run can bind the mavlink UDP ports (px4 14005 / mavros 14006)
  # AND the ROS master (11311). CRITICAL: killing gzserver/rosmaster alone is NOT enough -- stale `px4`
  # (SITL) and `mavros_node` survive the tmux teardown (px4 detaches; killProcessRecursive misses it) and
  # keep holding their UDP ports, so the NEXT run's mavros never connects (State: DISARMED NO_GPS, "Have
  # not received Mavros state") -> UAV never ready -> eval_data_node times out -> run produces no data.
  # DIRECT docker exec — NOT `bash -lc 'pkill'`: a login shell silently no-ops the pkills, so stale px4/
  # mavros survive between runs, the next take-off fails, and the run logs full CSV at ~0% coverage (a
  # "good" run that is actually dead). See memory long-batch-takeoff-degradation.
  docker exec "$CONTAINER" tmux -L mrs kill-server 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x px4 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f mavros 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x gzserver 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x gzclient 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x rosmaster 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f roscore 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f "roslaunch mrs" 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f "roslaunch motion_planning" 2>/dev/null
  docker exec "$CONTAINER" rm -f "$ONE_CTR/current_config.env" 2>/dev/null
  sleep 4   # let the UDP ports (14005/14006) and master port fully release before the next launch
}

echo "=========================================================="
echo "SUPERVISOR: target $TARGET_RUNS good '$LABEL' runs (SIM_TIME=${SIM_TIME}s)"
echo "=========================================================="
mkdir -p "$LABELDIR"
purge_partials
echo ">>> starting with $(good_runs)/$TARGET_RUNS good runs"

while [ "$(good_runs)" -lt "$TARGET_RUNS" ]; do
  have=$(good_runs)
  attempt=0
  while : ; do
    attempt=$((attempt + 1))
    if [ "$attempt" -gt "$MAX_ATTEMPTS_PER_RUN" ]; then
      echo ">>> ABORT: $MAX_ATTEMPTS_PER_RUN failed attempts for run $((have + 1)); container keeps dropping."
      echo ">>> final: $(good_runs)/$TARGET_RUNS good runs. Investigate the container-exit cause."
      exit 1
    fi
    ensure_container
    pre=$(ls -1d "$LABELDIR"/2* 2>/dev/null | sort)
    echo ">>> [$LABEL] run $((have + 1))/$TARGET_RUNS — attempt $attempt ($(date +%H:%M:%S))"
    docker exec -e EXP_CONFIG_SPEC="$SPEC" -e AEP_EARLY_STOP="${AEP_EARLY_STOP:-false}" -e AEP_EARLY_STOP_GRACE="${AEP_EARLY_STOP_GRACE:-60.0}" -e VOXEL_SIZE="${VOXEL_SIZE:-0.2}" -w "$ONE_CTR" "$CONTAINER" bash -lc "./run_experiments.sh explore 1 $SIM_TIME"
    rc=$?
    post=$(ls -1d "$LABELDIR"/2* 2>/dev/null | sort)
    newdir=$(comm -13 <(printf '%s\n' "$pre") <(printf '%s\n' "$post") | tail -1)
    cup=$(docker ps --filter name="$CONTAINER" --format '{{.Names}}')
    rows=0; [ -n "$newdir" ] && rows=$(csv_rows "$newdir")
    crashed=0; [ -n "$newdir" ] && [ -f "$newdir/.crashed" ] && crashed=1
    if [ -n "$newdir" ] && [ "$rows" -gt 1 ] && [ "$crashed" = 0 ]; then
      echo ">>> SUCCESS: $(basename "$newdir") rows=$rows maps=$(ls "$newdir/voxblox_maps" 2>/dev/null | wc -l)"
      break
    fi
    [ "$crashed" = 1 ] && echo ">>> CRASHED (disarmed mid-run = wall hit) — discarding & retrying"
    echo ">>> DROP (rc=$rc container='${cup:-DOWN}' newdir='${newdir:-none}' rows=$rows crashed=$crashed) — discarding & retrying"
    [ -n "$newdir" ] && [ -d "$newdir" ] && rm -rf "$newdir"
  done
done

echo "=========================================================="
echo ">>> DONE: $(good_runs)/$TARGET_RUNS good '$LABEL' runs"
ls -1d "$LABELDIR"/2* 2>/dev/null | while read -r d; do echo "   $(basename "$d"): rows=$(csv_rows "$d") maps=$(ls "$d/voxblox_maps" 2>/dev/null | wc -l)"; done
echo "=========================================================="
