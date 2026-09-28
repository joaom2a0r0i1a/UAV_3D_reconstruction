#!/bin/bash
# Host-side supervisor for run_experiments.sh, restarts the container and retries dropped runs
# Usage ./supervise_runs.sh [LABEL] [TARGET_RUNS] [SIM_TIME] [CONFIG_SPEC]

CONTAINER=noetic_ws
LABEL="${1:-GPU_marg_RRT}"
TARGET_RUNS="${2:-2}"
SIM_TIME="${3:-950}"
SPEC="${4:-}"
MAX_ATTEMPTS_PER_RUN=5

ONE_CTR=/home/ros1/ros1_motion_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone
DATA_HOST=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/single/motion_planning/data
LABELDIR="$DATA_HOST/$LABEL"

csv_rows() { wc -l <"$1/voxblox_data.csv" 2>/dev/null || echo 0; }

good_runs() {
  local n=0 d
  for d in "$LABELDIR"/2*; do
    [ -d "$d" ] || continue
    [ "$(csv_rows "$d")" -gt 1 ] && n=$((n + 1))
  done
  echo "$n"
}

# Remove unfinished run dirs of this label
purge_partials() {
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
  # Kill stale px4, mavros and rosmaster so the next run can bind its ports
  # Direct docker exec, a login shell silently skips pkill
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
  sleep 4
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
  while :; do
    attempt=$((attempt + 1))
    if [ "$attempt" -gt "$MAX_ATTEMPTS_PER_RUN" ]; then
      echo ">>> ABORT: $MAX_ATTEMPTS_PER_RUN failed attempts for run $((have + 1)); container keeps dropping."
      echo ">>> final: $(good_runs)/$TARGET_RUNS good runs. Investigate the container-exit cause."
      exit 1
    fi
    ensure_container
    pre=$(ls -1d "$LABELDIR"/2* 2>/dev/null | sort)
    echo ">>> [$LABEL] run $((have + 1))/$TARGET_RUNS — attempt $attempt ($(date +%H:%M:%S))"
    # Forward every campaign setting to the container
    fwd=()
    for v in $(compgen -e | grep -E '^(PLANNER_|AEP_|RH_NBVP_|VOXEL_)'); do fwd+=(-e "$v=${!v}"); done
    docker exec "${fwd[@]}" -e EXP_CONFIG_SPEC="$SPEC" -e AEP_EARLY_STOP="${AEP_EARLY_STOP:-false}" -e AEP_EARLY_STOP_GRACE="${AEP_EARLY_STOP_GRACE:-60.0}" -e VOXEL_SIZE="${VOXEL_SIZE:-0.2}" -w "$ONE_CTR" "$CONTAINER" bash -lc "./run_experiments.sh explore 1 $SIM_TIME"
    rc=$?
    post=$(ls -1d "$LABELDIR"/2* 2>/dev/null | sort)
    newdir=$(comm -13 <(printf '%s\n' "$pre") <(printf '%s\n' "$post") | tail -1)
    cup=$(docker ps --filter name="$CONTAINER" --format '{{.Names}}')
    rows=0
    [ -n "$newdir" ] && rows=$(csv_rows "$newdir")
    crashed=0
    [ -n "$newdir" ] && [ -f "$newdir/.crashed" ] && crashed=1
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
