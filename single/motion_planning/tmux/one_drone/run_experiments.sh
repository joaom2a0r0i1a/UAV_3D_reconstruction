#!/bin/bash
#
# run_experiments.sh — sweep the AEP planner over several configurations,
# running NUM_RUNS repetitions of each. Modeled on multi_start.sh.
#
# Each config is pushed to the planner via env vars written to
# ./current_config.env, which session.yml sources in its pre_window. The four
# planner knobs map to AEPlanner.yaml as:
#   AEP_RRT_STAR       -> local_planning/rrt_star     (false=RRT, true=RRT*)
#   AEP_MARGINAL_GAIN  -> evaluation/marginal_gain    (false=absolute, true=marginal/hash)
#   AEP_COMPUTE        -> evaluation/compute          (cpu | gpu)
#   AEP_MARGINAL_SPLIT -> evaluation/marginal_split   (false=fused, true=split)
#   AEP_BENCHMARK      -> evaluation/benchmark        (time+compare all methods)
# EXP_DATA_DIR routes each config's runs into data/<label>/<timestamp>/ so that
# full_voxblox_eval.launch (multi_series:=true) can compare methods.
#
# Usage:
#   ./run_experiments.sh [explore|benchmark] [num_runs] [sim_time_seconds]
# Examples:
#   ./run_experiments.sh                 # explore, 3 runs, 1850 s each
#   ./run_experiments.sh explore 5       # explore, 5 runs
#   ./run_experiments.sh benchmark 1 600 # 1 benchmark run of 10 min
#
# To add the other 4 planners to the exploration sweep, just uncomment the
# corresponding lines in EXPLORE_CONFIGS below.

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=mrs

# ------------------------------------------------------------------ knobs ----
MODE="${1:-explore}"          # explore | benchmark
NUM_RUNS="${2:-3}"            # repetitions per config (was 5 originally)
SIM_TIME="${3:-950}"         # seconds per run (wall-clock kill)
CHECK_INTERVAL=30            # seconds between liveness checks

# data/ lives two levels up from this script (motion_planning/data)
DATA_ROOT=$(readlink -f "$SCRIPTPATH/../../data")
ENV_FILE="$SCRIPTPATH/current_config.env"

# --------------------------------------------------------- experiment matrix -
# Format: label:rrt_star:marginal_gain:compute:marginal_split:benchmark
EXPLORE_CONFIGS=(
  # "GPU_abs_RRT:false:false:gpu:false:false"   # done earlier (2 good runs kept)
  "GPU_marg_RRT:false:true:gpu:false:false"
  # --- uncomment to add the other four planners ---
  # "CPU_abs_RRT:false:false:cpu:false:false"
  # "CPU_hash_RRT:false:true:cpu:false:false"
  # "GPU_abs_RRTstar:true:false:gpu:false:false"
  # "GPU_marg_RRTstar:true:true:gpu:false:false"
)

# Benchmark mode: all methods are timed/compared on the same tree per planning
# iteration, so a single benchmark=true run yields the B/C/D comparisons.
# (Full per-method CSV columns + v4 timing still require the AEPlanner.cpp
#  changes noted in the plan — this just enables the instrumentation.)
BENCHMARK_CONFIGS=(
  "BENCH_gpu_marg:false:true:gpu:false:true"
)

# ------------------------------------------------------------------ helpers --
write_env() {
  # $1=rrt_star $2=marginal_gain $3=compute $4=split $5=benchmark $6=datadir $7=time_limit_min
  cat > "$ENV_FILE" <<EOF
export AEP_RRT_STAR=$1
export AEP_MARGINAL_GAIN=$2
export AEP_COMPUTE=$3
export AEP_MARGINAL_SPLIT=$4
export AEP_BENCHMARK=$5
export EXP_DATA_DIR=$6
export EXP_TIME_LIMIT=$7
export AEP_EARLY_STOP=${AEP_EARLY_STOP:-false}
export AEP_EARLY_STOP_GRACE=${AEP_EARLY_STOP_GRACE:-60.0}
export NBV_X1_CSV=${NBV_X1_CSV:-}
export VOXEL_SIZE=${VOXEL_SIZE:-0.2}
EOF
}

terminate_sim() {
  echo ""
  echo ">>> Killing tmux session ($TMUX_SESSION_NAME)..."
  tmux -L $TMUX_SOCKET_NAME split-window -t $TMUX_SESSION_NAME
  tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME "sleep 1; tmux list-panes -s -F \"#{pane_pid} #{pane_current_command}\" | grep -v tmux | cut -d\" \" -f1 | while read in; do killProcessRecursive \$in; done; exit" ENTER
  sleep 2
  echo ">>> Simulation stopped."
}

cleanup() {
  # Remove the sourced config so a subsequent plain start.sh is unaffected.
  rm -f "$ENV_FILE"
}

on_sigint() {
  echo ""
  echo ">>> Ctrl+C — aborting sweep."
  terminate_sim
  cleanup
  exit 1
}
trap on_sigint SIGINT

run_config() {
  local spec="$1"
  local label rrt marg compute split bench
  IFS=':' read -r label rrt marg compute split bench <<< "$spec"

  local datadir="$DATA_ROOT/$label"
  mkdir -p "$datadir"
  local tlimit=$(( SIM_TIME / 60 ))   # eval_data_node self-stop, in minutes

  echo "=========================================================="
  echo "CONFIG: $label"
  echo "  rrt_star=$rrt marginal_gain=$marg compute=$compute split=$split benchmark=$bench"
  echo "  data -> $datadir   ($NUM_RUNS runs x ${SIM_TIME}s)"
  echo "=========================================================="

  for i in $(seq 1 "$NUM_RUNS"); do
    echo ">>> [$label] starting run #$i / $NUM_RUNS"
    write_env "$rrt" "$marg" "$compute" "$split" "$bench" "$datadir" "$tlimit"

    # Early-stop sentinel: eval_data_node writes this once it has flushed the CSV
    # and finalized the bag (only when AEP_EARLY_STOP=true). Clear it before start.
    local sentinel="$datadir/.run_complete"
    rm -f "$sentinel"

    tmuxinator start -p ./session.yml

    local elapsed=0
    while [ $elapsed -lt "$SIM_TIME" ]; do
      tmux -L $TMUX_SOCKET_NAME has-session -t $TMUX_SESSION_NAME 2>/dev/null
      if [ $? -ne 0 ]; then
        echo "Tmux session disappeared! Assuming emergency stop. Exiting."
        terminate_sim
        cleanup
        exit 1
      fi
      if [ -f "$sentinel" ]; then
        echo ">>> [$label] run self-completed early (planner terminated); tearing down."
        rm -f "$sentinel"
        break
      fi
      sleep $CHECK_INTERVAL
      elapsed=$((elapsed + CHECK_INTERVAL))
    done

    echo ">>> [$label] terminating run #$i"
    terminate_sim
    sleep 5
  done
}

# ------------------------------------------------------------------- driver --
echo "To emergency stop inside tmux: Ctrl+a then k then 9."
echo "To stop the whole sweep: Ctrl+C"
echo "MODE=$MODE NUM_RUNS=$NUM_RUNS SIM_TIME=$SIM_TIME DATA_ROOT=$DATA_ROOT"

case "$MODE" in
  explore)
    if [ -n "$EXP_CONFIG_SPEC" ]; then
      CONFIGS=("$EXP_CONFIG_SPEC")   # single spec injected by supervisor (overrides array)
    else
      CONFIGS=("${EXPLORE_CONFIGS[@]}")
    fi
    ;;
  benchmark) CONFIGS=("${BENCHMARK_CONFIGS[@]}") ;;
  *) echo "Unknown MODE '$MODE' (use: explore | benchmark)"; exit 2 ;;
esac

for spec in "${CONFIGS[@]}"; do
  run_config "$spec"
done

cleanup
echo ""
echo "All configs completed."
echo "Next: process results."
if [ "$MODE" = "explore" ]; then
  echo "  roslaunch motion_planning full_voxblox_eval.launch multi_series:=true"
  echo "  (ensure eval_plotting_node.py's multi_series label list matches the swept labels)"
else
  echo "  python3 ../../src/evaluate/process_benchmark.py   # once benchmark CSV columns are extended"
fi
