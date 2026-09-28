#!/bin/bash
# Parameterised experiment driver, campaign files under campaigns/
# Usage ./run_campaign.sh [--dry-run | --eval-only | --no-eval] [-N runs] [-T seconds] campaigns/<name>.conf
set -u
cd "$(dirname "$0")" || exit 1

DRY=0
DO_RUN=1
DO_EVAL=1
N_OVR=""
T_OVR=""
CONF=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --eval-only) DO_RUN=0 ;;
    --no-eval) DO_EVAL=0 ;;
    -N)
      shift
      N_OVR="$1"
      ;;
    -T)
      shift
      T_OVR="$1"
      ;;
    -h | --help)
      sed -n '2,30p' "$0"
      exit 0
      ;;
    -*)
      echo "unknown flag $1"
      exit 2
      ;;
    *) CONF="$1" ;;
  esac
  shift
done
[ -n "$CONF" ] && [ -f "$CONF" ] || {
  echo "usage: $0 [--dry-run|--eval-only|--no-eval] [-N n] [-T s] <campaign.conf>"
  exit 2
}

REPO=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction
DATA="$REPO/single/motion_planning/data"
LOGDIR="$REPO/single/motion_planning/tmux/one_drone/variants_logs"

# ---- Campaign Defaults ----
# Empty T uses the per-world budget
WORLD=school
PLANNER=aep
N=10
T=""
EARLY_STOP=false
GRACE=60.0
VOXEL_SIZE="${VOXEL_SIZE:-0.2}"
KEEP_FOLDER=multi_series_evaluation
SUMMARY=DECISION_SUMMARY.txt
RESTORE=true
MAP_KEEP=5
CONDITIONS=()
# shellcheck disable=SC1090
source "$CONF"
[ -n "$N_OVR" ] && N="$N_OVR"
[ -n "$T_OVR" ] && T="$T_OVR"
[ ${#CONDITIONS[@]} -gt 0 ] || {
  echo "campaign has no CONDITIONS"
  exit 2
}

# ---- Config Files, Temp Copies under --dry-run ----
if [ "$DRY" = 1 ]; then
  TMPD=$(mktemp -d)
  cp "$REPO/single/motion_planning/config/AEP.yaml" "$TMPD/AEP.yaml"
  cp "$REPO/single/motion_planning/config/RH_NBVP.yaml" "$TMPD/RH_NBVP.yaml"
  cp "$REPO/core/gain_evaluation/config/GainConfig.yaml" "$TMPD/GainConfig.yaml"
  cp "$REPO/core/cache_nodes/config/config.yaml" "$TMPD/cache_config.yaml"
  cp "$REPO/single/motion_planning/tmux/one_drone/session.yml" "$TMPD/session.yml"
  YAML="$TMPD/AEP.yaml"
  NYAML="$TMPD/RH_NBVP.yaml"
  GCFG="$TMPD/GainConfig.yaml"
  SESS="$TMPD/session.yml"
  CCFG="$TMPD/cache_config.yaml"
  LOGDIR="$TMPD"
  echo "### DRY-RUN — editing temp copies in $TMPD, no launches, no real writes ###"
else
  YAML="$REPO/single/motion_planning/config/AEP.yaml"
  NYAML="$REPO/single/motion_planning/config/RH_NBVP.yaml"
  GCFG="$REPO/core/gain_evaluation/config/GainConfig.yaml"
  SESS="$REPO/single/motion_planning/tmux/one_drone/session.yml"
  CCFG="$REPO/core/cache_nodes/config/config.yaml"
fi
mkdir -p "$LOGDIR"
SUMMARY="$LOGDIR/$SUMMARY"

# shellcheck disable=SC1091
source "$REPO/single/motion_planning/tmux/one_drone/lib_campaign.sh"

# T from the conf, -T or the world default, plus the startup tolerance
STARTUP_TOL_S=50
[ -n "$T" ] || T=$(($(world_time_limit "$WORLD") * 60 + STARTUP_TOL_S))
export VOXEL_SIZE

echo "==========================================================" | tee "$SUMMARY"
log "CAMPAIGN $(basename "$CONF"): world=$WORLD planner=$PLANNER N=$N T=${T}s voxel=${VOXEL_SIZE}m early_stop=$EARLY_STOP" | tee -a "$SUMMARY"
log "conditions: ${#CONDITIONS[@]}  keep=$KEEP_FOLDER" | tee -a "$SUMMARY"
echo "==========================================================" | tee -a "$SUMMARY"

# ---- Environment ----
set_world "$WORLD"
set_planner "$PLANNER"

# ---- Label Lists ----
ALL_LABELS=""
LABELS_CSV=""
for c in "${CONDITIONS[@]}"; do
  lbl="${c%%|*}"
  ALL_LABELS="$ALL_LABELS $lbl"
  LABELS_CSV="${LABELS_CSV:+$LABELS_CSV,}$lbl"
done
ALL_LABELS="${ALL_LABELS# }"

# ---- Run Each Condition ----
if [ "$DO_RUN" = 1 ]; then
  for c in "${CONDITIONS[@]}"; do
    IFS='|' read -r f1 f2 f3 f4 f5 f6 f7 f8 f9 <<<"$c"
    label="$f1"
    gain="$f2"
    if [ "$PLANNER" = aep ]; then
      rrt="${f4:-false}"
      set_aep_gain "$gain"
      export AEP_OBJECTIVE="${f5:-expdecay}"
      log "AEP objective=${f5:-expdecay}"
      # Optional node set f6 f7 f8, empty keeps the world default
      if [ -n "$f6" ]; then
        export PLANNER_AEP_NMAX="$f6" PLANNER_AEP_NTERM="$f7" PLANNER_AEP_NMIN="$f8"
        log "AEP node-set, N_max=$f6 N_termination=$f7 N_min_nodes=$f8"
      fi
      preflight "$WORLD" "$PLANNER"
      case "$gain" in marg) mspec=true ;; *) mspec=false ;; esac
      run_cond "$label" "$mspec" "$rrt"
    else
      nmax="$f3"
      nterm="$f4"
      step="$f5"
      fixed="$f6"
      optyaw="${f7:-true}"
      objective="${f8:-expdecay}"
      horizon="${f9:-1}"
      set_nbvp "$optyaw" "$nmax" "$nterm" "$step" "$fixed" "$objective" "$horizon"
      preflight "$WORLD" "$PLANNER"
      case "$gain" in marg) mspec=true ;; *) mspec=false ;; esac
      run_cond "$label" "$mspec" false
    fi
  done
else
  log "(--eval-only: skipping runs)"
fi

# ---- Restore Defaults ----
if [ "$RESTORE" = true ] && [ "$DRY" != 1 ]; then
  set_world school
  set_planner aep
  set_aep_gain marg
  log "repo restored -> school / AEP marginal default"
fi

# ---- Evaluation ----
if [ "$DO_EVAL" = 1 ]; then
  eval_campaign
else
  log "(--no-eval: skipping evaluation)"
fi

if [ "$DRY" = 1 ]; then
  echo
  echo "### DRY-RUN diffs (temp copies vs repo) ###"
  for f in AEP.yaml RH_NBVP.yaml GainConfig.yaml cache_config.yaml session.yml; do
    case "$f" in
      GainConfig.yaml) real="$REPO/core/gain_evaluation/config/$f" ;;
      cache_config.yaml) real="$REPO/core/cache_nodes/config/config.yaml" ;;
      session.yml) real="$REPO/single/motion_planning/tmux/one_drone/$f" ;;
      *) real="$REPO/single/motion_planning/config/$f" ;;
    esac
    d=$(diff "$real" "$TMPD/$f" 2>/dev/null)
    [ -n "$d" ] && {
      echo "--- $f ---"
      echo "$d"
    }
  done
  rm -rf "$TMPD"
fi

log "=== CAMPAIGN DONE — summary at $SUMMARY ==="
