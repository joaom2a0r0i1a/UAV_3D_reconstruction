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

# Settings reach the stack as environment variables, no config file is edited
log(){ echo "[$(date +%H:%M:%S)] $*"; }
die(){ echo "[$(date +%H:%M:%S)] FATAL: $*" >&2; exit 1; }
LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- World Tables ----
ALL_WORLDS="school police warehouse multistory big_maze maze"
world_marker(){ case "$1" in
  school)     echo "School" ;;
  police)     echo "Police Station" ;;
  warehouse)  echo "Warehouse" ;;
  multistory) echo "MultiStory" ;;
  big_maze)   echo "BigMaze" ;;
  maze)       echo "Maze" ;;
  *) die "unknown world '$1'";; esac; }
world_gt(){ case "$1" in
  school)     echo '$(rospack find uav_gazebo_environments)/ground_truth/school.ply' ;;
  police)     echo '$(rospack find uav_gazebo_environments)/ground_truth/police.ply' ;;
  warehouse)  echo '$(rospack find uav_gazebo_environments)/ground_truth/warehouse.ply' ;;
  multistory) echo '$(rospack find uav_gazebo_environments)/ground_truth/multistory.ply' ;;
  # big_maze ground truth not generated yet, volume evaluation only
  big_maze)   echo '$(rospack find uav_gazebo_environments)/ground_truth/big_maze.ply' ;;
  # maze ground truth not generated, volume evaluation only
  maze)       echo '$(rospack find uav_gazebo_environments)/ground_truth/maze.ply' ;;
  *) die "unknown world '$1'";; esac; }
# Planner clearance, under half the tightest passage and over the 0.348 m f450 footprint
world_uav_radius(){ case "$1" in
  school|police) echo "1.5" ;;
  warehouse)     echo "0.8" ;;
  multistory)    echo "0.5" ;;
  big_maze)      echo "0.8" ;;
  maze)          echo "0.7" ;;
  *) die "unknown world '$1'";; esac; }
# AEP gain threshold g_zero, lower means more frontiers
world_g_zero(){ case "$1" in
  school|police|big_maze|maze) echo "5.0" ;;
  warehouse) echo "3.0" ;;
  multistory) echo "2.0" ;;
  *) die "unknown world '$1'";; esac; }
# AEP waypoint advance distance, smaller cuts fewer corners
world_wp_reach(){ case "$1" in
  school|police) echo "0.8" ;;
  warehouse|multistory|big_maze) echo "0.5" ;;
  maze) echo "0.4" ;;
  *) die "unknown world '$1'";; esac; }
# AEP sizing, N_max N_termination N_min_nodes radius step_size tolerance
world_aep_params(){ case "$1" in
  school|police|maze) echo "50 300 300 3.0 2.0 1.5" ;;
  warehouse)     echo "250 1000 1000 2.0 1.5 1.0" ;;
  multistory)    echo "400 1600 1600 2.0 1.0 1.0" ;;
  big_maze)      echo "250 1000 1000 2.0 1.5 1.0" ;;
  *) die "unknown world '$1'";; esac; }
# RH_NBVP sizing, N_max N_termination radius step_size tolerance
world_nbv_params(){ case "$1" in
  school|police|maze) echo "50 300 2.0 2.0 0.5" ;;
  warehouse)     echo "250 1000 2.0 1.5 1.0" ;;
  multistory)    echo "400 1600 2.0 1.0 1.0" ;;
  big_maze)      echo "250 1000 2.0 1.5 1.0" ;;
  *) die "unknown world '$1'";; esac; }
# Exploration time budget per world in minutes
world_time_limit(){ case "$1" in
  school)     echo "30" ;;
  police)     echo "30" ;;
  warehouse)  echo "45" ;;
  multistory) echo "35" ;;
  big_maze)   echo "45" ;;
  maze)       echo "10" ;;
  *) die "unknown world '$1'";; esac; }
# Eval mode per world, gt scores observed ground-truth surface points, volume scores observed volume in the box
world_eval_mode(){ case "$1" in
  school|police) echo "gt" ;;
  warehouse|big_maze|multistory|maze) echo "volume" ;;
  *) die "unknown world '$1'";; esac; }

# Set the world, arg school | police | warehouse | multistory | big_maze | maze
set_world(){
  local w="$1"
  export PLANNER_ENV="$w"
  source "$LIB_DIR/environment.sh"
  [ -n "$PLANNER_WORLD_FILE" ] || die "unknown world '$w'"
  export PLANNER_UAV_RADIUS="$(world_uav_radius "$w")"
  export PLANNER_G_ZERO="$(world_g_zero "$w")"
  export PLANNER_WP_REACH="$(world_wp_reach "$w")"
  read -r PLANNER_AEP_NMAX PLANNER_AEP_NTERM PLANNER_AEP_NMIN \
          PLANNER_AEP_RADIUS PLANNER_AEP_STEP PLANNER_AEP_TOL <<< "$(world_aep_params "$w")"
  read -r PLANNER_RH_NBVP_NMAX PLANNER_RH_NBVP_NTERM \
          PLANNER_RH_NBVP_RADIUS PLANNER_RH_NBVP_STEP PLANNER_RH_NBVP_TOL <<< "$(world_nbv_params "$w")"
  export PLANNER_AEP_NMAX PLANNER_AEP_NTERM PLANNER_AEP_NMIN PLANNER_AEP_RADIUS PLANNER_AEP_STEP PLANNER_AEP_TOL
  export PLANNER_RH_NBVP_NMAX PLANNER_RH_NBVP_NTERM PLANNER_RH_NBVP_RADIUS PLANNER_RH_NBVP_STEP PLANNER_RH_NBVP_TOL
  log "world -> $w ($PLANNER_WORLD_FILE, spawn $PLANNER_SPAWN, uav_radius=$PLANNER_UAV_RADIUS, AEP N=$PLANNER_AEP_NMAX/$PLANNER_AEP_NTERM, RH_NBVP N=$PLANNER_RH_NBVP_NMAX/$PLANNER_RH_NBVP_NTERM)"
}

# ---- Planner Switch ----
# Set the planner, arg aep | nbvp | kaep | krhnbvp
set_planner(){
  case "$1" in
    aep)     export PLANNER_KIND=aep ;;
    nbvp)    export PLANNER_KIND=rhnbvp ;;
    kaep)    export PLANNER_KIND=kaep ;;
    krhnbvp) export PLANNER_KIND=krhnbvp ;;
    *) die "unknown planner '$1'";;
  esac
  log "planner -> $PLANNER_KIND"
}

# ---- AEP Gain Flags ----
# Set the AEP gain, arg abs | marg
set_aep_gain(){
  case "$1" in
    abs|control) export AEP_MARGINAL_GAIN=false ;;
    marg)        export AEP_MARGINAL_GAIN=true ;;
    *) die "unknown gain mode '$1' (use abs|marg)";;
  esac
  log "AEP gain=$1 (marginal_gain=$AEP_MARGINAL_GAIN)"
}

# Set RH_NBVP, args optyaw nmax nterm step fixed objective horizon
set_nbvp(){
  [ "$3" -gt "$2" ] || die "N_termination ($3) MUST be > N_max ($2) or receding-horizon never recedes"
  export RH_NBVP_OPTIMIZE_YAW="$1"
  export PLANNER_RH_NBVP_NMAX="$2"
  export PLANNER_RH_NBVP_NTERM="$3"
  export PLANNER_RH_NBVP_STEP="$4"
  export RH_NBVP_FIXED_STEP="$5"
  export RH_NBVP_OBJECTIVE="${6:-expdecay}"
  export RH_NBVP_HORIZON="${7:-1}"
  log "RH_NBVP optimize_yaw=$1 N_max=$2 N_term=$3 step=$4 fixed_step=$5 objective=${6:-expdecay} horizon=${7:-1}"
}

# ---- Preflight Checks ----
# Check the live config, args world planner
preflight(){
  local w="$1" p="$2" kind
  [ "$PLANNER_ENV" = "$w" ]                          || die "environment is '$PLANNER_ENV' not '$w'"
  [ "$PLANNER_WORLD_FILE" = "$(env_value "$w" world)" ] || die "world file is '$PLANNER_WORLD_FILE' not $w"
  [ "$PLANNER_SPAWN" = "$(env_value "$w" spawn)" ]      || die "spawn is '$PLANNER_SPAWN' not $w"
  case "$p" in aep) kind=aep;; nbvp) kind=rhnbvp;; kaep) kind=kaep;; krhnbvp) kind=krhnbvp;; esac
  [ "$PLANNER_KIND" = "$kind" ]                      || die "planner is '$PLANNER_KIND' not '$kind'"
  log "PREFLIGHT OK, $w / $p"
}

# Complete run, arg run dir: the eval node's own stop line (not a startup failure), no .crashed, a CSV data row (ADR 0019)
complete_run(){
  local s
  s=$(grep "Stopping the experiment:" "$1/data_log.txt" 2>/dev/null | tail -1)
  [ -n "$s" ] && [[ $s != *"startup failed"* ]] && [ ! -f "$1/.crashed" ] &&
    [ "$(wc -l < "$1/voxblox_data.csv" 2>/dev/null || echo 0)" -gt 2 ]
}

# ---- One Condition Run ----
# Run one condition, args label marginal_gain rrt_star
run_cond(){
  local label="$1" marg="$2" rrt="${3:-false}"
  local spec="${label}:${rrt}:${marg}:gpu:false:false"
  log "===== $label START (target N=$N x ${T}s, early_stop=$EARLY_STOP, spec=$spec) ====="
  if [ "$DRY" = 1 ]; then
    echo "       DRY: AEP_EARLY_STOP=$EARLY_STOP AEP_EARLY_STOP_GRACE=$GRACE \\"
    echo "            supervise \"$label\" \"$N\" \"$T\" \"$spec\""
    return 0
  fi
  ( AEP_EARLY_STOP="$EARLY_STOP" AEP_EARLY_STOP_GRACE="$GRACE" supervise "$label" "$N" "$T" "$spec" ) >>"$LOGDIR/run_${label}.log" 2>&1 || log "WARN supervise $label returned nonzero"
  log "===== $label DONE ====="
}

# ---- Evaluation ----
# Evaluate the campaign labels
eval_campaign(){
  local GT GTCFG L EM EV=true VOL=false
  GT=$(world_gt "$WORLD"); GTCFG="$WORLD"
  # evaluate:=true makes the plotting node take the GT branch, so volume worlds run evaluate:=false
  EM=$(world_eval_mode "$WORLD"); [ "$EM" = volume ] && { EV=false; VOL=true; }
  local suffix="${KEEP_FOLDER#multi_series_}"
  log "=== EVAL stage-1 per label ($WORLD GT=$GTCFG eval_mode=$EM evaluate=$EV evaluate_volume=$VOL) ==="
  if [ "$DRY" = 1 ]; then
    for L in $ALL_LABELS; do echo "       DRY: eval stage-1 $L (target_directory=data/$L gt=$GT cfg=$GTCFG eval_mode=$EM evaluate=$EV evaluate_volume=$VOL)"; done
    echo "       DRY: eval stage-2 multi_series series_labels=$LABELS_CSV eval_mode=$EM evaluate=$EV evaluate_volume=$VOL -> mv multi_series_evaluation -> $KEEP_FOLDER"
    return 0
  fi
  docker start noetic_ws >/dev/null 2>&1; sleep 4
  for L in $ALL_LABELS; do
    docker exec -e MPLBACKEND=Agg noetic_ws bash -lc '
      source /home/ros1/voxblox_ws/devel/setup.bash; source /home/ros1/ros1_motion_ws/devel/setup.bash
      roslaunch motion_planning full_voxblox_eval.launch \
        target_directory:=$(rospack find motion_planning)/data/'"$L"' method:=all multi_series:=false \
        evaluate:='"$EV"' evaluate_volume:='"$VOL"' \
        gt_file_path:='"$GT"' environment:='"$GTCFG"'' \
      >"$LOGDIR/stage1_${L}_${suffix}.log" 2>&1
    log "  stage-1 $L done"
    # Thin the voxblox maps, keep every MAP_KEEP-th and the last
    docker exec noetic_ws bash -lc '
      source /home/ros1/ros1_motion_ws/devel/setup.bash
      python3 $(rospack find motion_planning)/scripts/evaluation/analysis/thin_maps.py $(rospack find motion_planning)/data/'"$L"' '"${MAP_KEEP:-5}"'' \
      >>"$LOGDIR/stage1_${L}_${suffix}.log" 2>&1
    log "  thinned maps (keep every ${MAP_KEEP:-5}th + last) for $L"
  done
  log "=== EVAL stage-2 multi_series ($WORLD GT) ==="
  docker exec -e MPLBACKEND=Agg noetic_ws bash -lc '
    source /home/ros1/voxblox_ws/devel/setup.bash; source /home/ros1/ros1_motion_ws/devel/setup.bash
    roslaunch motion_planning full_voxblox_eval.launch \
      target_directory:=$(rospack find motion_planning)/data \
      multi_series:=true series_labels:="'"$LABELS_CSV"'" \
      evaluate:='"$EV"' evaluate_volume:='"$VOL"' \
      gt_file_path:='"$GT"' environment:='"$GTCFG"'' \
    >"$LOGDIR/stage2_${suffix}.log" 2>&1
  rm -rf "$DATA/$KEEP_FOLDER"
  mv "$DATA/multi_series_evaluation" "$DATA/$KEEP_FOLDER" 2>/dev/null
  rm -f "$DATA/$KEEP_FOLDER/".~lock* 2>/dev/null
  log "  saved render -> data/$KEEP_FOLDER/"
  {
    echo ""; echo "===== COMPLETION-TIME MILESTONES (min to %known) ====="
    grep -iE "Timing corresponding" "$LOGDIR/stage2_${suffix}.log"
    echo ""; echo "Good-run tally (target $N):"
    for L in $ALL_LABELS; do
      local n=0 d
      for d in "$DATA/$L"/2*; do [ -d "$d" ] && complete_run "$d" && n=$((n+1)); done
      echo "  $L: $n/$N good"
    done
    echo ""; echo "Plot: data/$KEEP_FOLDER/MultiSeriesOverview.png"
    echo "IF A RUN STALLS: do NOT delete it -> python3 ../../scripts/evaluation/analysis/stall_forensics.py <its tmp_bags/*.bag> $((T/60))"
  } | tee -a "$SUMMARY"
}

# Host-side supervisor for run_experiments.sh, restarts the container and retries dropped runs, args LABEL TARGET_RUNS SIM_TIME CONFIG_SPEC
supervise() {
CONTAINER=noetic_ws
LABEL="${1:-GPU_marg_RRT}"
TARGET_RUNS="${2:-2}"
SIM_TIME="${3:-950}"
SPEC="${4:-}"
MAX_ATTEMPTS_PER_RUN=5

ONE_CTR=/home/ros1/ros1_motion_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone
DATA_HOST=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/single/motion_planning/data
LABELDIR="$DATA_HOST/$LABEL"

csv_rows() { wc -l <"$1/voxblox_data.csv" 2>/dev/null || echo 0; }

good_runs() {
  local n=0 d
  for d in "$LABELDIR"/2*; do
    [ -d "$d" ] || continue
    complete_run "$d" && n=$((n + 1))
  done
  echo "$n"
}

# Remove unfinished run dirs of this label
purge_partials() {
  local d
  for d in "$LABELDIR"/2*; do
    [ -d "$d" ] || continue
    if ! complete_run "$d"; then rm -rf "$d" && echo ">>> purged partial $(basename "$d")"; fi
  done
}

ensure_container() {
  if [ -z "$(docker ps --filter name="$CONTAINER" --format '{{.Names}}')" ]; then
    echo ">>> container down — starting $CONTAINER"
    docker start "$CONTAINER" >/dev/null 2>&1
    sleep 6
  fi
  # Kill stale px4, mavros and rosmaster so the next run can bind its ports
  # Direct docker exec, a login shell silently skips pkill
  docker exec "$CONTAINER" tmux -L mrs kill-server 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x px4 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f mavros 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x gzserver 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x gzclient 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -x rosmaster 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f roscore 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f "roslaunch mrs" 2>/dev/null
  docker exec "$CONTAINER" pkill -9 -f "roslaunch motion_planning" 2>/dev/null
  docker exec "$CONTAINER" rm -f "$ONE_CTR/current_config.env" 2>/dev/null
  sleep 4
}

echo "=========================================================="
echo "SUPERVISOR: target $TARGET_RUNS good '$LABEL' runs (SIM_TIME=${SIM_TIME}s)"
echo "=========================================================="
mkdir -p "$LABELDIR"
purge_partials
echo ">>> starting with $(good_runs)/$TARGET_RUNS good runs"

while [ "$(good_runs)" -lt "$TARGET_RUNS" ]; do
  have=$(good_runs)
  attempt=0
  while :; do
    attempt=$((attempt + 1))
    if [ "$attempt" -gt "$MAX_ATTEMPTS_PER_RUN" ]; then
      echo ">>> ABORT: $MAX_ATTEMPTS_PER_RUN failed attempts for run $((have + 1)); container keeps dropping."
      echo ">>> final: $(good_runs)/$TARGET_RUNS good runs. Investigate the container-exit cause."
      exit 1
    fi
    ensure_container
    pre=$(ls -1d "$LABELDIR"/2* 2>/dev/null | sort)
    echo ">>> [$LABEL] run $((have + 1))/$TARGET_RUNS — attempt $attempt ($(date +%H:%M:%S))"
    # Forward every campaign setting to the container
    fwd=()
    for v in $(compgen -e | grep -E '^(PLANNER_|AEP_|RH_NBVP_|VOXEL_)'); do fwd+=(-e "$v=${!v}"); done
    docker exec "${fwd[@]}" -e EXP_CONFIG_SPEC="$SPEC" -e AEP_EARLY_STOP="${AEP_EARLY_STOP:-false}" -e AEP_EARLY_STOP_GRACE="${AEP_EARLY_STOP_GRACE:-60.0}" -e VOXEL_SIZE="${VOXEL_SIZE:-0.2}" -w "$ONE_CTR" "$CONTAINER" bash -lc "./run_experiments.sh explore 1 $SIM_TIME"
    rc=$?
    post=$(ls -1d "$LABELDIR"/2* 2>/dev/null | sort)
    newdir=$(comm -13 <(printf '%s\n' "$pre") <(printf '%s\n' "$post") | tail -1)
    cup=$(docker ps --filter name="$CONTAINER" --format '{{.Names}}')
    rows=0
    [ -n "$newdir" ] && rows=$(csv_rows "$newdir")
    crashed=0
    [ -n "$newdir" ] && [ -f "$newdir/.crashed" ] && crashed=1
    if [ -n "$newdir" ] && complete_run "$newdir"; then
      echo ">>> SUCCESS: $(basename "$newdir") rows=$rows maps=$(ls "$newdir/voxblox_maps" 2>/dev/null | wc -l)"
      break
    fi
    [ "$crashed" = 1 ] && echo ">>> CRASHED (disarmed mid-run = wall hit) — discarding & retrying"
    echo ">>> DROP (rc=$rc container='${cup:-DOWN}' newdir='${newdir:-none}' rows=$rows crashed=$crashed) — discarding & retrying"
    [ -n "$newdir" ] && [ -d "$newdir" ] && rm -rf "$newdir"
  done
done

echo "=========================================================="
echo ">>> DONE: $(good_runs)/$TARGET_RUNS good '$LABEL' runs"
ls -1d "$LABELDIR"/2* 2>/dev/null | while read -r d; do echo "   $(basename "$d"): rows=$(csv_rows "$d") maps=$(ls "$d/voxblox_maps" 2>/dev/null | wc -l)"; done
echo "=========================================================="
}

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
