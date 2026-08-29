#!/bin/bash
# parallel_eval.sh — stage-1 GT eval of FINISHED runs, run CONCURRENTLY, each on its own ISOLATED
# roscore (ports 11320+), so it never touches a live sim (which owns roscore 11311). Each run is evaluated
# via full_voxblox_eval method:=single (evaluate:=true evaluate_volume:=false), which fills the GT
# reconstruction/coverage columns (MeanError/RMSE/UnknownVoxels) of that run's voxblox_data.csv against the
# ground-truth pointcloud. SKIPS runs still recording (CSV <= 1 row) and runs already evaluated (CSV > 4 cols).
# Niced, so a running simulation keeps CPU priority. Run INSIDE the container.
#   Usage: MAXJ=<n> parallel_eval.sh [experiment_config.yaml] [gt_ply] <label_or_run_dir> [more dirs...]
#   Defaults are set for the SCHOOL environment (School.yaml + gt_school_processed.ply).
set -u
GTCFG="${1:-School.yaml}"
GTPLY="${2:-$(rospack find motion_planning)/data/gt_school_processed.ply}"
shift 2
MAXJ="${MAXJ:-3}"          # concurrent evals (kept modest so a live sim is undisturbed)

# Collect finished, un-evaluated run dirs across the given label dirs.
runs=()
for arg in "$@"; do
  for d in "$arg"/2* "$arg"; do          # accept label dirs (2*) or a single run dir
    [ -d "$d" ] || continue
    csv="$d/voxblox_data.csv"; [ -f "$csv" ] || continue
    rows=$(wc -l <"$csv" 2>/dev/null || echo 0)
    cols=$(head -1 "$csv" 2>/dev/null | awk -F, '{print NF}')
    [ "$rows" -gt 1 ]       || { echo "  skip (still recording) $(basename "$d")"; continue; }
    [ "${cols:-0}" -le 4 ]  || { echo "  skip (already evaluated) $(basename "$d")"; continue; }
    runs+=("$d")
  done
done
echo ">>> parallel_eval: ${#runs[@]} runs to evaluate, up to $MAXJ concurrent (isolated roscores)"

i=0
for d in "${runs[@]}"; do
  while [ "$(jobs -rp | wc -l)" -ge "$MAXJ" ]; do sleep 2; done
  p=$((11320 + i)); i=$((i + 1))
  (
    export ROS_MASTER_URI="http://localhost:$p"
    roscore -p "$p" >"/tmp/pe_rc_$p.log" 2>&1 & rc=$!
    sleep 5
    MPLBACKEND=Agg timeout 600 nice -n 15 roslaunch motion_planning full_voxblox_eval.launch \
      target_directory:="$d" method:=single multi_series:=false \
      evaluate:=true evaluate_volume:=false \
      gt_file_path:="$GTPLY" experiment_config:="$GTCFG" \
      >"/tmp/pe_$(basename "$d").log" 2>&1
    kill "$rc" 2>/dev/null
    cols=$(head -1 "$d/voxblox_data.csv" | awk -F, '{print NF}')
    echo "  done $(basename "$d")  (cols=$cols)"
  ) &
done
wait
echo ">>> parallel_eval DONE"
