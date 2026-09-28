#!/bin/bash
# Parallel ground truth evaluation of finished runs, each on its own roscore
# Usage MAXJ=<n> parallel_eval.sh [environment] [gt_ply] <label_or_run_dir> [more dirs...]
set -u
ENVIRONMENT="${1:-school}"
GTPLY="${2:-$(rospack find uav_gazebo_environments)/ground_truth/$ENVIRONMENT.ply}"
shift 2
MAXJ="${MAXJ:-3}"

# Finished runs not yet evaluated
runs=()
for arg in "$@"; do
  for d in "$arg"/2* "$arg"; do
    [ -d "$d" ] || continue
    csv="$d/voxblox_data.csv"
    [ -f "$csv" ] || continue
    rows=$(wc -l <"$csv" 2>/dev/null || echo 0)
    cols=$(head -1 "$csv" 2>/dev/null | awk -F, '{print NF}')
    [ "$rows" -gt 1 ] || {
      echo "  skip (still recording) $(basename "$d")"
      continue
    }
    [ "${cols:-0}" -le 4 ] || {
      echo "  skip (already evaluated) $(basename "$d")"
      continue
    }
    runs+=("$d")
  done
done
echo ">>> parallel_eval: ${#runs[@]} runs to evaluate, up to $MAXJ concurrent (isolated roscores)"

i=0
for d in "${runs[@]}"; do
  while [ "$(jobs -rp | wc -l)" -ge "$MAXJ" ]; do sleep 2; done
  p=$((11320 + i))
  i=$((i + 1))
  (
    export ROS_MASTER_URI="http://localhost:$p"
    roscore -p "$p" >"/tmp/pe_rc_$p.log" 2>&1 &
    rc=$!
    sleep 5
    MPLBACKEND=Agg timeout 600 nice -n 15 roslaunch motion_planning full_voxblox_eval.launch \
      target_directory:="$d" method:=single multi_series:=false \
      evaluate:=true evaluate_volume:=false \
      gt_file_path:="$GTPLY" environment:="$ENVIRONMENT" \
      >"/tmp/pe_$(basename "$d").log" 2>&1
    kill "$rc" 2>/dev/null
    cols=$(head -1 "$d/voxblox_data.csv" | awk -F, '{print NF}')
    echo "  done $(basename "$d")  (cols=$cols)"
  ) &
done
wait
echo ">>> parallel_eval DONE"
