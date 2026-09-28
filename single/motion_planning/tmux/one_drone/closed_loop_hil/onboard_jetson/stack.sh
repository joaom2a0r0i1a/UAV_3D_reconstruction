#!/bin/bash
# Orin HIL stack, voxblox, RH_NBVP planner and cache in one tmux session
# Env in VOXEL_SIZE, HIL_LOG, EXP_DATA_DIR
S=hilorin
L=mrs
: "${VOXEL_SIZE:=0.2}"
: "${HIL_LOG:=$HOME/hilB_logs/planner.log}"
: "${EXP_DATA_DIR:=$HOME/hilB_data}"
mkdir -p "$(dirname "$HIL_LOG")" "$EXP_DATA_DIR"
: >"$HIL_LOG"

ENV="source ~/hil_env.sh; source ~/jm_ws/devel/setup.bash; export VOXEL_SIZE=$VOXEL_SIZE; export EXP_DATA_DIR=$EXP_DATA_DIR"
T="tmux -L $L"
send() { $T send-keys -t "$S:$1" "$ENV; $2" Enter; }

$T kill-session -t $S 2>/dev/null

$T new-session -d -s $S -n voxblox
send voxblox 'roslaunch motion_planning processed_voxblox.launch'
$T new-window -t $S -n planner
send planner "AEP_BENCHMARK=true AEP_MARGINAL_GAIN=true RH_NBVP_BENCH_SUITE=timing roslaunch motion_planning planner.launch planner:=rhnbvp 2>&1 | tee $HIL_LOG"
$T new-window -t $S -n cache
send cache 'roslaunch cache_nodes cache.launch'

echo ">>> Orin stack up (session '$S'), VOXEL_SIZE=$VOXEL_SIZE, log -> $HIL_LOG"
