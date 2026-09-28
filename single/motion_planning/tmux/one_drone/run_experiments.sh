#!/bin/bash
# Sweep the AEP planner over configurations, NUM_RUNS each
# Settings go through ./current_config.env, sourced by session.yml
# Usage ./run_experiments.sh [explore | benchmark] [num_runs] [sim_time_seconds]

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=mrs

# ---- Knobs ----
MODE="${1:-explore}"
NUM_RUNS="${2:-3}"
SIM_TIME="${3:-950}"
CHECK_INTERVAL=30

# data/ two levels up
DATA_ROOT=$(readlink -f "$SCRIPTPATH/../../data")
ENV_FILE="$SCRIPTPATH/current_config.env"
OVERRIDES_FILE="$SCRIPTPATH/current_overrides.yaml"

# ---- Experiment Matrix ----
# Format label:rrt_star:marginal_gain:compute:marginal_split:benchmark
EXPLORE_CONFIGS=(
  # "GPU_abs_RRT:false:false:gpu:false:false"   # done earlier (2 good runs kept)
  "GPU_marg_RRT:false:true:gpu:false:false"
  # Other four planners
)

# Benchmark mode, all methods timed on the same tree
BENCHMARK_CONFIGS=(
  "BENCH_gpu_marg:false:true:gpu:false:true"
)

# ---- Helpers ----
write_overrides() {
  # Campaign parameters for the planner as yaml, only the variables that are set
  add() { [ -n "$2" ] && echo "$1 $2"; }
  {
    [ "$PLANNER_KIND" = aep ] || [ "$PLANNER_KIND" = rhnbvp ] && add path/uav_radius "$PLANNER_UAV_RADIUS"
    if [ "$PLANNER_KIND" = aep ]; then
      add path/waypoint_reach_distance "$PLANNER_WP_REACH"
      add local_planning/g_zero "$PLANNER_G_ZERO"
      add local_planning/N_max "$PLANNER_AEP_NMAX"
      add local_planning/N_termination "$PLANNER_AEP_NTERM"
      add local_planning/radius "$PLANNER_AEP_RADIUS"
      add local_planning/step_size "$PLANNER_AEP_STEP"
      add local_planning/tolerance "$PLANNER_AEP_TOL"
      add global_planning/N_min_nodes "$PLANNER_AEP_NMIN"
      add evaluation/objective "$AEP_OBJECTIVE"
    fi
    if [ "$PLANNER_KIND" = rhnbvp ]; then
      add rrt/N_max "$PLANNER_RH_NBVP_NMAX"
      add rrt/N_termination "$PLANNER_RH_NBVP_NTERM"
      add rrt/radius "$PLANNER_RH_NBVP_RADIUS"
      add rrt/step_size "$PLANNER_RH_NBVP_STEP"
      add rrt/tolerance "$PLANNER_RH_NBVP_TOL"
      add rrt/fixed_step "$RH_NBVP_FIXED_STEP"
      add rrt/execution_horizon "$RH_NBVP_HORIZON"
      add evaluation/optimize_yaw "$RH_NBVP_OPTIMIZE_YAW"
      add evaluation/objective "$RH_NBVP_OBJECTIVE"
    fi
  } | sort -s -t/ -k1,1 | awk '{ split($1, k, "/"); if (k[1] != s) { print k[1] ":"; s = k[1] } print "  " k[2] ": " $2 }'
}

write_env() {
  # Args (unused) marginal_gain compute split benchmark datadir time_limit_min
  cat >"$ENV_FILE" <<EOF
export AEP_MARGINAL_GAIN=$2
export AEP_COMPUTE=$3
export AEP_MARGINAL_SPLIT=$4
export AEP_BENCHMARK=$5
export EXP_DATA_DIR=$6
export EXP_TIME_LIMIT=$7
export AEP_EARLY_STOP=${AEP_EARLY_STOP:-false}
export AEP_EARLY_STOP_GRACE=${AEP_EARLY_STOP_GRACE:-60.0}
export RH_NBVP_ACCURACY_CSV=${RH_NBVP_ACCURACY_CSV:-}
export VOXEL_SIZE=${VOXEL_SIZE:-0.2}
export PLANNER_ENV=${PLANNER_ENV:-school}
export PLANNER_UAV_RADIUS=${PLANNER_UAV_RADIUS:-}
export PLANNER_G_ZERO=${PLANNER_G_ZERO:-}
export PLANNER_WP_REACH=${PLANNER_WP_REACH:-}
export PLANNER_AEP_NMAX=${PLANNER_AEP_NMAX:-}
export PLANNER_AEP_NTERM=${PLANNER_AEP_NTERM:-}
export PLANNER_AEP_NMIN=${PLANNER_AEP_NMIN:-}
export PLANNER_AEP_RADIUS=${PLANNER_AEP_RADIUS:-}
export PLANNER_AEP_STEP=${PLANNER_AEP_STEP:-}
export PLANNER_AEP_TOL=${PLANNER_AEP_TOL:-}
export PLANNER_RH_NBVP_NMAX=${PLANNER_RH_NBVP_NMAX:-}
export PLANNER_RH_NBVP_NTERM=${PLANNER_RH_NBVP_NTERM:-}
export PLANNER_RH_NBVP_RADIUS=${PLANNER_RH_NBVP_RADIUS:-}
export PLANNER_RH_NBVP_STEP=${PLANNER_RH_NBVP_STEP:-}
export PLANNER_RH_NBVP_TOL=${PLANNER_RH_NBVP_TOL:-}
export PLANNER_KIND=${PLANNER_KIND:-rhnbvp}
export RH_NBVP_OPTIMIZE_YAW=${RH_NBVP_OPTIMIZE_YAW:-}
export RH_NBVP_FIXED_STEP=${RH_NBVP_FIXED_STEP:-}
export RH_NBVP_OBJECTIVE=${RH_NBVP_OBJECTIVE:-}
export RH_NBVP_HORIZON=${RH_NBVP_HORIZON:-}
export AEP_OBJECTIVE=${AEP_OBJECTIVE:-}
EOF
  # Campaign overrides for the planner, loaded by planner.launch
  (
    source "$ENV_FILE"
    write_overrides
  ) >"$OVERRIDES_FILE"
  [ -s "$OVERRIDES_FILE" ] && echo "export PLANNER_OVERRIDES=$OVERRIDES_FILE" >>"$ENV_FILE"
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
  # Remove the sourced config and overrides
  rm -f "$ENV_FILE" "$OVERRIDES_FILE"
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
  IFS=':' read -r label rrt marg compute split bench <<<"$spec"

  local datadir="$DATA_ROOT/$label"
  mkdir -p "$datadir"
  local tlimit=$((SIM_TIME / 60))

  echo "=========================================================="
  echo "CONFIG: $label"
  echo "  rrt_star=$rrt marginal_gain=$marg compute=$compute split=$split benchmark=$bench"
  echo "  data -> $datadir   ($NUM_RUNS runs x ${SIM_TIME}s)"
  echo "=========================================================="

  for i in $(seq 1 "$NUM_RUNS"); do
    echo ">>> [$label] starting run #$i / $NUM_RUNS"
    write_env "$rrt" "$marg" "$compute" "$split" "$bench" "$datadir" "$tlimit"

    # Clear the early-stop sentinel
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

# ---- Driver ----
echo "To emergency stop inside tmux: Ctrl+a then k then 9."
echo "To stop the whole sweep: Ctrl+C"
echo "MODE=$MODE NUM_RUNS=$NUM_RUNS SIM_TIME=$SIM_TIME DATA_ROOT=$DATA_ROOT"

case "$MODE" in
  explore)
    if [ -n "$EXP_CONFIG_SPEC" ]; then
      CONFIGS=("$EXP_CONFIG_SPEC")
    else
      CONFIGS=("${EXPLORE_CONFIGS[@]}")
    fi
    ;;
  benchmark) CONFIGS=("${BENCHMARK_CONFIGS[@]}") ;;
  *)
    echo "Unknown MODE '$MODE' (use: explore | benchmark)"
    exit 2
    ;;
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
  echo "  benchmark: grep '\[bench_batch_check]' / '\[timing' lines from the run's rosout log"
fi
