#!/bin/bash
# Offline stage 2 evaluation, volume only. Run as eval_real.sh <experiment_dir> [label ...].
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RW_PKG="$(cd "$SCRIPT_DIR/../.." && pwd)"                       # motion_planning_real_world
MP="$(cd "$RW_PKG/../motion_planning" && pwd)"                  # motion_planning (sim pkg)
EVAL_CONFIG="${EVAL_CONFIG:-Basketball.yaml}"
ROS_PORT="${ROS_PORT:-11351}"
export MPLBACKEND=Agg

EXP_DIR="${1:?usage: eval_real.sh <experiment_dir> [label ...]}"
EXP_DIR="$(cd "$EXP_DIR" && pwd)"
shift || true

# Labels: args, else subdirs containing timestamped runs.
if [ "$#" -gt 0 ]; then
  LABELS=("$@")
else
  LABELS=()
  for d in "$EXP_DIR"/*/; do
    [ -d "$d" ] || continue
    name="$(basename "$d")"
    case "$name" in multi_series_evaluation|series_evaluation|tmp_bags|graphs) continue;; esac
    ls -d "$d"2*_* >/dev/null 2>&1 && LABELS+=("$name")
  done
fi
[ "${#LABELS[@]}" -gt 0 ] || { echo "no labels with runs found under $EXP_DIR"; exit 1; }
echo "[eval_real] experiment: $EXP_DIR  labels: ${LABELS[*]}"

export ROS_MASTER_URI="http://localhost:$ROS_PORT"
roscore -p "$ROS_PORT" > /tmp/eval_real_roscore.log 2>&1 &
ROSCORE_PID=$!
trap 'kill $ROSCORE_PID 2>/dev/null' EXIT
sleep 4

run_eval_launch() {  # $1 target_dir, $2 method, $3 multi_series, extra log: $4
  roslaunch motion_planning_real_world evaluate_plot_real.launch \
    target_directory:="$1" method:="$2" multi_series:="$3" \
    experiment_config:="$EVAL_CONFIG" 2>&1 | tee -a "$4"
}

# ---- Stage A: per-run evaluation (skip already-evaluated runs) ----
for L in "${LABELS[@]}"; do
  for RUN in "$EXP_DIR/$L"/2*_*/; do
    [ -d "$RUN" ] || continue
    if [ -f "$RUN/graphs/SimulationOverview.png" ]; then
      echo "[eval_real] $RUN already evaluated, skipping"; continue
    fi
    echo "[eval_real] evaluating run $RUN"
    run_eval_launch "${RUN%/}" single false "$EXP_DIR/eval_runs.log"
  done
done

# ---- Stage B: multi-series over the experiment dir (milestones printed to stdout) ----
MS_LOG="$EXP_DIR/eval_multiseries.log"
: > "$MS_LOG"
echo "[eval_real] multi-series over $EXP_DIR"
run_eval_launch "$EXP_DIR" single true "$MS_LOG"

# ---- Stage C: milestones table + path/velocity + RESULTS.txt skeleton ----
python3 "$MP/scripts/evaluation/analysis/milestones_from_log.py" "$MS_LOG" "$EXP_DIR/milestones.txt" || true

# absolute label paths so results can live outside the repo
PV_ARGS=()
for L in "${LABELS[@]}"; do PV_ARGS+=("$EXP_DIR/$L"); done
PV_LOG="$EXP_DIR/pathvel.log"
MP="$MP" OUT="$EXP_DIR/pathvel.json" PV_TOPIC=/mavros/local_position/pose \
  python3 "$MP/scripts/evaluation/analysis/path_vel_mapped.py" "${PV_ARGS[@]}" 2>&1 | tee "$PV_LOG" || true

RESULTS="$EXP_DIR/RESULTS.txt"
{
  echo "REAL-WORLD RESULTS — $(basename "$EXP_DIR")  (generated $(date +%F) by eval_real.sh)"
  echo "Volume-based coverage; box: $EVAL_CONFIG (+ per-run offset.txt shift); no ground truth."
  echo
  echo "=== MILESTONES (from eval_plotting_node stdout — canonical) ==="
  cat "$EXP_DIR/milestones.txt" 2>/dev/null || echo "(milestones.txt missing)"
  echo
  echo "=== PATH LENGTH / AVERAGE VELOCITY (path_vel_mapped, mavros pose bags) ==="
  grep -E "^### |-> N=" "$PV_LOG" 2>/dev/null || echo "(pathvel.log missing)"
  echo
  echo "=== TERMINATION ==="
  echo "NA (fill from planner data_log/termination analysis if applicable)"
} > "$RESULTS"
echo "[eval_real] wrote $RESULTS"
echo "[eval_real] done. Optional: thin maps afterwards with"
echo "  MAP_KEEP=5 python3 $MP/scripts/evaluation/analysis/thin_maps.py $EXP_DIR"
