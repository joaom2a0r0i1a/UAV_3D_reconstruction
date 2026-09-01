#!/bin/bash
# ONE HIL Path-B X2 config on the ORIN (RH-NBVP, yaw-opt).  Args:  VOXEL(0.2|0.1)  N_MAX
# Methodology mirrors data/timing/X2_desktop: timing_after_s=600 (10-min sim-time delay), collect
# NEED=10 whole-tree captures (nodes>=0.7*N). Leaves ~/hilB_logs/timing_yawopt_<vt>_n<N>.log.
# Precondition: the DESKTOP sim is already running FRESH and the ROS link is up (~/hil_env.sh).
V=${1:?voxel 0.2|0.1}; N=${2:?N_max}
ROOT=~/jm_ws/src/UAV_3D_reconstruction/single/motion_planning
YAML=$ROOT/config/NBVPlanner.yaml ; HERE=$ROOT/tmux/one_drone/closed_loop_hil/onboard_jetson
VT=$(echo "$V" | sed 's/^0\./0p/;s/^\./0p/')          # 0.2->0p2  0.1->0p1
LOG=~/hilB_logs/timing_yawopt_${VT}_n${N}.log ; mkdir -p ~/hilB_logs
NEED=10 ; XMAX=25 ; THR=$(awk "BEGIN{printf \"%d\", 0.7*$N}")   # collect 10 mature captures; benchmark up to 25 replans
NTERM=$(awk "BEGIN{v=2*$N; if(v<300)v=300; printf \"%d\", v}")  # >N_max REQUIRED or RH-NBVP self-terminates on any zero-gain iter
TAFTER=${TAFTER:-600}     # sim-time delay before benchmarking (600 = the real 10-min methodology; lower for a quick chain test)

# --- yaml: RH-NBVP yaw-opt X2 timing, N_max=N, 10-min delay ---
sed -i -E \
  -e 's/^  optimize_yaw:.*/  optimize_yaw: true/'   -e 's/^  marginal_gain:.*/  marginal_gain: true/' \
  -e 's/^  compute:.*/  compute: "gpu"/'            -e 's/^  suite:.*/  suite: "x2"/' \
  -e 's/^  enabled:.*/  enabled: true/' \
  -e "s/^  N_max:.*/  N_max: $N/"                   -e "s/^  N_termination:.*/  N_termination: $NTERM/" \
  -e "s/^  timing_after_s:.*/  timing_after_s: ${TAFTER}.0/"  -e "s/^  x2_max:.*/  x2_max: $XMAX/" \
  -e 's/^  recovery_enabled:.*/  recovery_enabled: true/'  -e 's/^  recovery_timeout:.*/  recovery_timeout: 900.0/' \
  "$YAML"    # KEEP recovery_enabled=true (its guard also stops the k-- collide-spin); raise recovery_timeout so the heavy N>=5000 build+benchmark cycle (~16s+ per replan, inside the plan window) never trips the time-based backtrack
echo ">>> [$VT N$N] yaml: N_max=$N N_termination=$NTERM timing_after_s=$TAFTER x2_max=$XMAX recovery_timeout=900 -> $LOG"

# --- launch stack (voxblox+planner+cache; planner tees to $LOG) + start mission ---
VOXEL_SIZE=$V HIL_LOG="$LOG" bash "$HERE/stack.sh"
source ~/jm_ws/devel/setup.bash    # ROS on PATH for rosservice (non-interactive ssh shell doesn't source it)
source ~/hil_env.sh
for i in $(seq 1 120); do rosservice list 2>/dev/null | grep -q '/uav1/planner_node/start' && break; sleep 2; done
sleep 2; rosservice call /uav1/planner_node/start 2>/dev/null || true
echo ">>> [$VT N$N] started; waiting for $NEED whole-tree captures (nodes>=$THR)  [600s sim-time delay first]"

# --- wait for 10 mature captures, or a wall-clock cap ---
mature(){ grep -a '\[X2rep\]' "$LOG" 2>/dev/null | grep -oE 'nodes=[0-9]+' | cut -d= -f2 | awk -v t=$THR '$1>=t' | wc -l; }
end=$((SECONDS+${WALLCAP:-2400}))    # wall cap per config (default 40min; raise via WALLCAP for heavy 0.1/N10000 accumulation)
while [ $SECONDS -lt $end ]; do
  m=$(mature)
  echo "  [$VT N$N] mature=$m/$NEED  (total X2rep=$(grep -ac '\[X2rep\]' "$LOG" 2>/dev/null))"
  [ "$m" -ge "$NEED" ] && { echo ">>> [$VT N$N] reached $m mature."; break; }
  sleep 20
done

# --- teardown Orin stack (graceful, then session, then targeted pkill — Orin is native OS) ---
for w in planner voxblox cache; do tmux -L mrs send-keys -t hilorin:$w C-c 2>/dev/null; done
sleep 8; tmux -L mrs kill-session -t hilorin 2>/dev/null
pkill -INT -f 'roslaunch motion_planning|roslaunch cache_nodes' 2>/dev/null; sleep 3
pkill -f 'voxblox_node|planner_node' 2>/dev/null; sleep 3
echo ">>> [$VT N$N] DONE  mature=$(mature)  ->  $LOG"
