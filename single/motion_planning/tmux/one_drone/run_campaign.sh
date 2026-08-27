#!/bin/bash
# =============================================================================
# run_campaign.sh — single parameterized experiment driver.
#
# Replaces the ~25 one-off drivers (school_*, police_*, nbvp_*, aep_*). Each of
# those is now a small campaign file under campaigns/ that sets a handful of
# variables + a CONDITIONS list; this driver applies the config, runs each
# condition to N good runs (via supervise_runs.sh), and evaluates.
#
# Usage:
#   ./run_campaign.sh campaigns/school_n10.conf            # full: configure+run+eval
#   ./run_campaign.sh --dry-run campaigns/school_n10.conf  # validate: no launch, no file writes
#   ./run_campaign.sh --eval-only campaigns/school_n10.conf# skip runs, just (re)evaluate
#   ./run_campaign.sh --no-eval  campaigns/school_n10.conf # runs only, no eval
#   ./run_campaign.sh -N 5 campaigns/school_n10.conf       # override target good runs
#   ./run_campaign.sh -T 950 campaigns/police_n10.conf     # override sim seconds
#
# Campaign file (sourced bash) — see campaigns/README.md:
#   WORLD=school|police   PLANNER=aep|nbvp
#   N=<target good runs>  T=<sim seconds>
#   EARLY_STOP=true|false GRACE=60.0
#   KEEP_FOLDER=multi_series_<name>   SUMMARY=DECISION_SUMMARY_<NAME>.txt
#   RESTORE=true|false    # restore repo to school/marginal default at end (default true)
#   CONDITIONS=(          # AEP : "label|gain[|_|rrt_star]"  gain=abs|marg  (field 3 unused)
#     ...                 # NBVP: "label|gain|nmax|nterm|step|fixed[|optyaw]"  gain=abs|marg
#   )
# =============================================================================
set -u
cd "$(dirname "$0")" || exit 1

DRY=0; DO_RUN=1; DO_EVAL=1; N_OVR=""; T_OVR=""
CONF=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --eval-only) DO_RUN=0 ;;
    --no-eval) DO_EVAL=0 ;;
    -N) shift; N_OVR="$1" ;;
    -T) shift; T_OVR="$1" ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    -*) echo "unknown flag $1"; exit 2 ;;
    *) CONF="$1" ;;
  esac
  shift
done
[ -n "$CONF" ] && [ -f "$CONF" ] || { echo "usage: $0 [--dry-run|--eval-only|--no-eval] [-N n] [-T s] <campaign.conf>"; exit 2; }

REPO=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction
DATA="$REPO/single/motion_planning/data"
LOGDIR="$REPO/single/motion_planning/tmux/one_drone/variants_logs"
mkdir -p "$LOGDIR"

# ---- campaign defaults, then load ----
# T="" => derive per-world budget from lib_campaign world_time_limit after the conf is sourced.
WORLD=school; PLANNER=aep; N=10; T=""; EARLY_STOP=false; GRACE=60.0
VOXEL_SIZE="${VOXEL_SIZE:-0.2}"   # voxblox resolution (auto-propagates map+planner+buffers); conf may override
KEEP_FOLDER=multi_series_evaluation; SUMMARY=DECISION_SUMMARY.txt; RESTORE=true
MAP_KEEP=5   # voxblox-map thinning: keep every MAP_KEEP-th map + the last (60s interval x5 = 5min); conf may override
CONDITIONS=()
# shellcheck disable=SC1090
source "$CONF"
[ -n "$N_OVR" ] && N="$N_OVR"
[ -n "$T_OVR" ] && T="$T_OVR"
SUMMARY="$LOGDIR/$SUMMARY"
[ ${#CONDITIONS[@]} -gt 0 ] || { echo "campaign has no CONDITIONS"; exit 2; }

# ---- config file targets: real files, or temp copies under --dry-run --------
if [ "$DRY" = 1 ]; then
  TMPD=$(mktemp -d)
  cp "$REPO/single/motion_planning/config/AEPlanner.yaml"      "$TMPD/AEPlanner.yaml"
  cp "$REPO/single/motion_planning/config/NBVPlanner.yaml"     "$TMPD/NBVPlanner.yaml"
  cp "$REPO/core/gain_evaluation/config/GainConfig.yaml"    "$TMPD/GainConfig.yaml"
  cp "$REPO/core/cache_nodes/config/config.yaml"             "$TMPD/cache_config.yaml"
  cp "$REPO/single/motion_planning/tmux/one_drone/session.yml" "$TMPD/session.yml"
  YAML="$TMPD/AEPlanner.yaml"; NYAML="$TMPD/NBVPlanner.yaml"
  GCFG="$TMPD/GainConfig.yaml"; SESS="$TMPD/session.yml"; CCFG="$TMPD/cache_config.yaml"
  echo "### DRY-RUN — editing temp copies in $TMPD, no launches, no real writes ###"
else
  YAML="$REPO/single/motion_planning/config/AEPlanner.yaml"
  NYAML="$REPO/single/motion_planning/config/NBVPlanner.yaml"
  GCFG="$REPO/core/gain_evaluation/config/GainConfig.yaml"
  SESS="$REPO/single/motion_planning/tmux/one_drone/session.yml"
  CCFG="$REPO/core/cache_nodes/config/config.yaml"
fi

# shellcheck disable=SC1091
source "$REPO/single/motion_planning/tmux/one_drone/lib_campaign.sh"

# T budget: conf/-T value if set, else the per-world default (minutes -> seconds).
# +STARTUP_TOL_S: eval_data_node's time_limit counts from sim-ready (t=0), but the planner only
# activates ~20-30 s later; without this tolerance we'd terminate before the planner finishes and
# drop the final map. (This is the original 1850s = 30min + 50s tolerance, now per-world.)
STARTUP_TOL_S=50
[ -n "$T" ] || T=$(( $(world_time_limit "$WORLD") * 60 + STARTUP_TOL_S ))
export VOXEL_SIZE   # forwarded to the container by supervise_runs.sh -> run_experiments.sh write_env

echo "==========================================================" | tee "$SUMMARY"
log "CAMPAIGN $(basename "$CONF"): world=$WORLD planner=$PLANNER N=$N T=${T}s voxel=${VOXEL_SIZE}m early_stop=$EARLY_STOP" | tee -a "$SUMMARY"
log "conditions: ${#CONDITIONS[@]}  keep=$KEEP_FOLDER" | tee -a "$SUMMARY"
echo "==========================================================" | tee -a "$SUMMARY"

# ---- environment (world + planner), once per campaign ----
set_world "$WORLD"
set_planner "$PLANNER"

# ---- build label lists for eval ----
ALL_LABELS=""; LABELS_CSV=""
for c in "${CONDITIONS[@]}"; do
  lbl="${c%%|*}"
  ALL_LABELS="$ALL_LABELS $lbl"
  LABELS_CSV="${LABELS_CSV:+$LABELS_CSV,}$lbl"
done
ALL_LABELS="${ALL_LABELS# }"

# ---- run each condition ----
if [ "$DO_RUN" = 1 ]; then
  for c in "${CONDITIONS[@]}"; do
    IFS='|' read -r f1 f2 f3 f4 f5 f6 f7 f8 f9 <<< "$c"
    label="$f1"; gain="$f2"
    if [ "$PLANNER" = aep ]; then
      rrt="${f4:-false}"   # f3 (legacy variant field) is ignored; absolute is always the path-union baseline
      set_aep_gain "$gain"
      set_key "$YAML" objective "${f5:-expdecay}"
      log "AEP objective=${f5:-expdecay}"
      # optional per-condition node-set (f6=N_max f7=N_termination f8=N_min_nodes); empty -> keep world default
      if [ -n "$f6" ]; then
        set_key "$YAML" N_max "$f6"; set_key "$YAML" N_termination "$f7"; set_key "$YAML" N_min_nodes "$f8"
        log "AEP node-set -> N_max=$f6 N_termination=$f7 N_min_nodes=$f8"
      fi
      preflight "$WORLD" "$PLANNER"
      case "$gain" in marg) mspec=true ;; *) mspec=false ;; esac
      run_cond "$label" "$mspec" "$rrt"
    else  # nbvp
      nmax="$f3"; nterm="$f4"; step="$f5"; fixed="$f6"; optyaw="${f7:-true}"; objective="${f8:-expdecay}"; horizon="${f9:-1}"
      set_nbvp "$optyaw" "$nmax" "$nterm" "$step" "$fixed" "$objective" "$horizon"
      preflight "$WORLD" "$PLANNER"
      case "$gain" in marg) mspec=true ;; *) mspec=false ;; esac
      run_cond "$label" "$mspec" false
    fi
  done
else
  log "(--eval-only: skipping runs)"
fi

# ---- restore repo to school / marginal default (as the originals did) ----
if [ "$RESTORE" = true ] && [ "$DRY" != 1 ]; then
  set_world school
  set_planner aep
  set_aep_gain marg
  log "repo restored -> school / AEP marginal default"
fi

# ---- eval ----
if [ "$DO_EVAL" = 1 ]; then
  eval_campaign
else
  log "(--no-eval: skipping evaluation)"
fi

if [ "$DRY" = 1 ]; then
  echo; echo "### DRY-RUN diffs (temp copies vs repo) ###"
  for f in AEPlanner.yaml NBVPlanner.yaml GainConfig.yaml cache_config.yaml session.yml; do
    case "$f" in
      GainConfig.yaml)   real="$REPO/core/gain_evaluation/config/$f" ;;
      cache_config.yaml) real="$REPO/core/cache_nodes/config/config.yaml" ;;
      session.yml)       real="$REPO/single/motion_planning/tmux/one_drone/$f" ;;
      *)                 real="$REPO/single/motion_planning/config/$f" ;;
    esac
    d=$(diff "$real" "$TMPD/$f" 2>/dev/null)
    [ -n "$d" ] && { echo "--- $f ---"; echo "$d"; }
  done
  rm -rf "$TMPD"
fi

log "=== CAMPAIGN DONE — summary at $SUMMARY ==="
