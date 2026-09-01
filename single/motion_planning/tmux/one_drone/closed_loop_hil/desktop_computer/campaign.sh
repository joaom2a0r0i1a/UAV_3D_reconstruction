#!/bin/bash
# HIL Path-B campaign MASTER (runs on the DESKTOP). Sweeps voxel x N_max; each config = fresh sim ->
# SSH the Jetson worker (onboard_jetson/run_config.sh) -> pull the log -> teardown sim. ~12 configs, multi-hour.
# PREREQS:
#   - copy config.sh.template -> config.sh (set ORIN + DESKTOP_IP for your setup)
#   - key-based SSH desktop->Jetson:   ssh-copy-id "$ORIN"
#   - Jetson has:  ~/hil_env.sh, and in jm_ws .../closed_loop_hil/onboard_jetson/:  stack.sh + run_config.sh
#   - desktop container 'noetic_ws' up, --network host, firewall allows the Jetson
_CFG="$(cd "$(dirname "$0")" && pwd)/../config.sh"
[ -f "$_CFG" ] && source "$_CFG"
: "${ORIN:?set ORIN in closed_loop_hil/config.sh (copy from config.sh.template)}"
: "${DESKTOP_IP:?set DESKTOP_IP in closed_loop_hil/config.sh (copy from config.sh.template)}"
CDIR='cd ~/ros1_motion_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone/closed_loop_hil/desktop_computer'
OUT=./raw_logs ; mkdir -p "$OUT"    # staging for pulled logs; move the keepers into data/timing/X2_jetson_hilB/ after
: "${VOXELS:=0.2 0.1}" ; : "${NS:=50 100 500 1000 5000 10000}"   # env-overridable to run a subset
CLOUD=/uav1/pcl_filter_rs_front_pitched/points_processed         # the topic voxblox consumes
DESK_SRC='source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/ros1_motion_ws/devel/setup.bash 2>/dev/null'
ORIN_SRC='source ~/hil_env.sh 2>/dev/null; source ~/jm_ws/devel/setup.bash 2>/dev/null'

teardown_sim(){   # SAFE: killProcessRecursive on pane pids (NO pkill — that stops the container)
  docker exec noetic_ws bash -lc '
    tmux -L mrs has-session -t simulation 2>/dev/null || exit 0
    tmux -L mrs split-window -t simulation
    tmux -L mrs send-keys -t simulation "sleep 1; tmux list-panes -s -F \"#{pane_pid} #{pane_current_command}\" | grep -v tmux | cut -d\" \" -f1 | while read in; do killProcessRecursive \$in; done; exit" ENTER
    sleep 10; tmux -L mrs kill-session -t simulation 2>/dev/null' 2>/dev/null
  sleep 5
}

# Readiness = the processed cloud is streaming on BOTH the PC and the Orin (same link, same topic).
# As soon as both are true, start the Orin worker — no blind fixed sleep.
desk_cloud(){ docker exec noetic_ws bash -lc "$DESK_SRC; timeout 5 rostopic hz $CLOUD 2>/dev/null | grep -m1 -q 'average rate'" >/dev/null 2>&1 && echo 1 || echo 0; }
orin_cloud(){ timeout 12 ssh -o ConnectTimeout=6 "$ORIN" "$ORIN_SRC; timeout 5 rostopic hz $CLOUD 2>/dev/null | grep -m1 -q 'average rate'" >/dev/null 2>&1 && echo 1 || echo 0; }
wait_link_ready(){
  local end=$((SECONDS+300)) d o
  while [ $SECONDS -lt $end ]; do
    d=$(desk_cloud); o=$(orin_cloud)
    echo "    link check: PC_cloud=$d ORIN_cloud=$o  (${SECONDS}s)"
    [ "$d" = 1 ] && [ "$o" = 1 ] && { echo "    -> cloud streaming on BOTH sides; starting Orin worker."; return 0; }
    sleep 6
  done
  echo "    !! cloud not streaming on both sides after 300s — starting worker anyway."; return 1
}

for V in $VOXELS; do for N in $NS; do
  VT=$(echo "$V" | sed 's/^0\./0p/')
  echo "==================== voxel=$V N_max=$N  ($(date +%H:%M:%S)) ===================="
  teardown_sim                                              # clean slate
  docker exec noetic_ws bash -lc "$CDIR && DESKTOP_IP=$DESKTOP_IP bash sim.sh"
  wait_link_ready
  echo "  -> Orin worker (blocks until 10 whole-tree captures or its 40-min cap)"
  ssh "$ORIN" "WALLCAP=${WALLCAP:-2400} bash ~/jm_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone/closed_loop_hil/onboard_jetson/run_config.sh $V $N"
  scp "$ORIN:~/hilB_logs/timing_yawopt_${VT}_n${N}.log" "$OUT/" 2>/dev/null \
     && echo "  pulled -> $OUT/timing_yawopt_${VT}_n${N}.log" || echo "  !! no log pulled for $VT N$N"
  teardown_sim
done; done

echo "CAMPAIGN DONE -> $OUT/.  Analyze per voxel (from motion_planning/):"
echo "  MPLBACKEND=Agg MP=\$PWD X2_LOG=$PWD/$OUT X2_TAG=timing_yawopt_0p2_n X2_SUFFIX=_hilB_0p2 python3 scripts/evaluation/figures/x2_online.py"
echo "  MPLBACKEND=Agg MP=\$PWD X2_LOG=$PWD/$OUT X2_TAG=timing_yawopt_0p1_n X2_SUFFIX=_hilB_0p1 python3 scripts/evaluation/figures/x2_online.py"
