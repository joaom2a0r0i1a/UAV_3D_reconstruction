#!/bin/bash
# ORIN HIL stack: voxblox + RH-NBVP planner (x2 suite) + cache, in one tmux session.
# The planner console is tee'd to $HIL_LOG (that's where [X2rep]/[X2cpu] land — rosout goes to
# the DESKTOP master, so we capture stdout locally instead).
# Env in:  VOXEL_SIZE (0.1/0.2), HIL_LOG (planner log path), EXP_DATA_DIR (must exist).
# Teardown:  tmux -L mrs kill-session -t hilorin   (campaign driver does this + a pkill sweep)
S=hilorin ; L=mrs
: "${VOXEL_SIZE:=0.2}"
: "${HIL_LOG:=$HOME/hilB_logs/planner.log}"
: "${EXP_DATA_DIR:=$HOME/hilB_data}"
mkdir -p "$(dirname "$HIL_LOG")" "$EXP_DATA_DIR"
: > "$HIL_LOG"   # truncate fresh for this config

ENV="source ~/hil_env.sh; source ~/jm_ws/devel/setup.bash; export VOXEL_SIZE=$VOXEL_SIZE; export EXP_DATA_DIR=$EXP_DATA_DIR"
T="tmux -L $L"
send(){ $T send-keys -t "$S:$1" "$ENV; $2" Enter; }

$T kill-session -t $S 2>/dev/null

$T new-session -d -s $S -n voxblox
send voxblox 'roslaunch motion_planning processed_voxblox.launch'
$T new-window -t $S -n planner
send planner "AEP_BENCHMARK=true AEP_MARGINAL_GAIN=true NBV_BENCH_SUITE=x2 roslaunch motion_planning NBVPlanner.launch 2>&1 | tee $HIL_LOG"
$T new-window -t $S -n cache
send cache 'roslaunch cache_nodes cache.launch'

echo ">>> Orin stack up (session '$S'), VOXEL_SIZE=$VOXEL_SIZE, log -> $HIL_LOG"
