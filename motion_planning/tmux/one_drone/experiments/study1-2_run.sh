#!/bin/bash
#
# study1-2_run.sh — unattended host orchestrator for Day A + Day B.
#   Day A: Study 1  AEP GPU abs RRT   vs marg RRT   (3 good runs each)
#   Day B: Study 2  AEP GPU abs RRT*  vs marg RRT*  (3 good runs each)
#
# Each label is driven by supervise_runs.sh (survives container drops). The exact
# config spec is injected via the 4th arg (EXP_CONFIG_SPEC) so no EXPLORE_CONFIGS
# hand-editing is needed. Labels run sequentially; a label that hits its retry cap
# is logged and the campaign continues to the next label.
#
# Usage: ./study1-2_run.sh   (SIM_TIME defaults 1850s)

SCRIPT=$(readlink -f "$0"); cd "$(dirname "$SCRIPT")"
SUP=../supervise_runs.sh
SIM_TIME="${1:-1850}"

# label : target_good_runs : "spec(label:rrt_star:marg:compute:split:bench)"
JOBS=(
  "GPU_abs_RRT:3:GPU_abs_RRT:false:false:gpu:false:false"
  "GPU_marg_RRT:3:GPU_marg_RRT:false:true:gpu:false:false"
  "GPU_abs_RRTstar:3:GPU_abs_RRTstar:true:false:gpu:false:false"
  "GPU_marg_RRTstar:3:GPU_marg_RRTstar:true:true:gpu:false:false"
)

echo "############################################################"
echo "# CAMPAIGN Day A+B start  SIM_TIME=${SIM_TIME}s"
echo "############################################################"

for job in "${JOBS[@]}"; do
  label="${job%%:*}"
  rest="${job#*:}"
  target="${rest%%:*}"
  spec="${rest#*:}"          # remaining = full "label:rrt:marg:compute:split:bench"

  echo ""
  echo "======================= $label (target $target) ======================="
  $SUP "$label" "$target" "$SIM_TIME" "$spec"
  rc=$?
  if [ $rc -ne 0 ]; then
    echo "!!! $label supervisor exited rc=$rc (retry cap hit?) — continuing to next label."
  else
    echo ">>> $label complete."
  fi
done

echo ""
echo "############################################################"
echo "# CAMPAIGN Day A+B finished. Good-run tally:"
DATA=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/data
for l in GPU_abs_RRT GPU_marg_RRT GPU_abs_RRTstar GPU_marg_RRTstar; do
  n=0; for d in "$DATA/$l"/2*; do [ -d "$d" ] && [ "$(wc -l < "$d/voxblox_data.csv" 2>/dev/null || echo 0)" -gt 1 ] && n=$((n+1)); done
  echo "#   $l: $n good runs"
done
echo "# Next: two-stage full_voxblox_eval (see EXPERIMENT_PLAN.md §6)."
echo "############################################################"
