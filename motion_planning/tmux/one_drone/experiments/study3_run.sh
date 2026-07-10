#!/bin/bash
#
# study3_run.sh — unattended host orchestrator for Study 3:
#   RH-NBVP GPU absolute RRT vs marginal RRT (3 good runs each).
#
# PREREQ (handled by the caller, not here):
#   - NBVPlanner.launch wired for AEP_* optenv overrides (done).
#   - session.yml motion_planner window switched to NBVPlanner.launch.
# Same supervisor + spec-injection mechanism as study1-2_run.sh.

SCRIPT=$(readlink -f "$0"); cd "$(dirname "$SCRIPT")"
SUP=../supervise_runs.sh
SIM_TIME="${1:-1850}"

JOBS=(
  "NBV_abs_RRT:3:NBV_abs_RRT:false:false:gpu:false:false"
  "NBV_marg_RRT:3:NBV_marg_RRT:false:true:gpu:false:false"
)

echo "############################################################"
echo "# CAMPAIGN Study 3 (RH-NBVP) start  SIM_TIME=${SIM_TIME}s"
echo "############################################################"

for job in "${JOBS[@]}"; do
  label="${job%%:*}"; rest="${job#*:}"
  target="${rest%%:*}"; spec="${rest#*:}"
  echo ""
  echo "======================= $label (target $target) ======================="
  $SUP "$label" "$target" "$SIM_TIME" "$spec"
  rc=$?
  [ $rc -ne 0 ] && echo "!!! $label supervisor exited rc=$rc — continuing." || echo ">>> $label complete."
done

echo ""
echo "############################################################"
DATA=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/data
for l in NBV_abs_RRT NBV_marg_RRT; do
  n=0; for d in "$DATA/$l"/2*; do [ -d "$d" ] && [ "$(wc -l < "$d/voxblox_data.csv" 2>/dev/null || echo 0)" -gt 1 ] && n=$((n+1)); done
  echo "#   $l: $n good runs"
done
echo "# CAMPAIGN Study 3 finished."
echo "############################################################"
