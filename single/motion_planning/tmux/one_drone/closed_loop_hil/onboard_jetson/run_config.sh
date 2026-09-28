#!/bin/bash
# One HIL Path B timing config on the Orin, args VOXEL N_MAX
V=${1:?voxel 0.2|0.1}; N=${2:?N_max}
ROOT=~/jm_ws/src/UAV_3D_reconstruction/single/motion_planning
YAML=$ROOT/config/RH_NBVP.yaml ; HERE=$ROOT/tmux/one_drone/closed_loop_hil/onboard_jetson
VT=$(echo "$V" | sed 's/^0\./0p/;s/^\./0p/')
LOG=~/hilB_logs/timing_yawopt_${VT}_n${N}.log ; mkdir -p ~/hilB_logs
NEED=10 ; XMAX=25 ; THR=$(awk "BEGIN{printf \"%d\", 0.7*$N}")
# N_termination must exceed N_max
NTERM=$(awk "BEGIN{v=2*$N; if(v<300)v=300; printf \"%d\", v}")
TAFTER=${TAFTER:-600}

# RH_NBVP yaw-opt timing suite yaml, recovery_enabled stays true
sed -i -E \
  -e 's/^  optimize_yaw:.*/  optimize_yaw: true/'   -e 's/^  marginal_gain:.*/  marginal_gain: true/' \
  -e 's/^  compute:.*/  compute: "gpu"/'            -e 's/^  suite:.*/  suite: "timing"/' \
  -e 's/^  enabled:.*/  enabled: true/' \
  -e "s/^  N_max:.*/  N_max: $N/"                   -e "s/^  N_termination:.*/  N_termination: $NTERM/" \
  -e "s/^  timing_after_s:.*/  timing_after_s: ${TAFTER}.0/"  -e "s/^  x2_max:.*/  x2_max: $XMAX/" \
  -e 's/^  recovery_enabled:.*/  recovery_enabled: true/'  -e 's/^  recovery_timeout:.*/  recovery_timeout: 900.0/' \
  "$YAML"
echo ">>> [$VT N$N] yaml: N_max=$N N_termination=$NTERM timing_after_s=$TAFTER x2_max=$XMAX recovery_timeout=900 -> $LOG"

# ---- Launch Stack ----
VOXEL_SIZE=$V HIL_LOG="$LOG" bash "$HERE/stack.sh"
source ~/jm_ws/devel/setup.bash
source ~/hil_env.sh
for i in $(seq 1 120); do rosservice list 2>/dev/null | grep -q '/uav1/planner_node/start' && break; sleep 2; done
sleep 2; rosservice call /uav1/planner_node/start 2>/dev/null || true
echo ">>> [$VT N$N] started; waiting for $NEED whole-tree captures (nodes>=$THR)  [600s sim-time delay first]"

# ---- Wait for 10 Captures ----
mature(){ grep -a '\[timing_marg\]' "$LOG" 2>/dev/null | grep -oE 'nodes=[0-9]+' | cut -d= -f2 | awk -v t=$THR '$1>=t' | wc -l; }
end=$((SECONDS+${WALLCAP:-2400}))
while [ $SECONDS -lt $end ]; do
  m=$(mature)
  echo "  [$VT N$N] mature=$m/$NEED  (total timing_marg=$(grep -ac '\[timing_marg\]' "$LOG" 2>/dev/null))"
  [ "$m" -ge "$NEED" ] && { echo ">>> [$VT N$N] reached $m mature."; break; }
  sleep 20
done

# Teardown, graceful first
for w in planner voxblox cache; do tmux -L mrs send-keys -t hilorin:$w C-c 2>/dev/null; done
sleep 8; tmux -L mrs kill-session -t hilorin 2>/dev/null
pkill -INT -f 'roslaunch motion_planning|roslaunch cache_nodes' 2>/dev/null; sleep 3
pkill -f 'voxblox_node|planner_node' 2>/dev/null; sleep 3
echo ">>> [$VT N$N] DONE  mature=$(mature)  ->  $LOG"
