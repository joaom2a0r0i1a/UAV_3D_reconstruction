#!/bin/bash
# Shared library for run_campaign.sh, sourced
# Settings reach the stack as environment variables, no config file is edited

log(){ echo "[$(date +%H:%M:%S)] $*"; }
die(){ echo "[$(date +%H:%M:%S)] FATAL: $*" >&2; exit 1; }
LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- World Tables ----
ALL_WORLDS="school police warehouse multistory big_maze"
world_marker(){ case "$1" in
  school)     echo "School" ;;
  police)     echo "Police Station" ;;
  warehouse)  echo "Warehouse" ;;
  multistory) echo "MultiStory" ;;
  big_maze)   echo "BigMaze" ;;
  *) die "unknown world '$1'";; esac; }
world_gt(){ case "$1" in
  school)     echo '$(rospack find motion_planning)/data/gt_school_processed.ply' ;;
  police)     echo '$(rospack find motion_planning)/data/gt_police_station_processed.ply' ;;
  warehouse)  echo '$(rospack find motion_planning)/data/gt_warehouse_processed.ply' ;;
  multistory) echo '$(rospack find motion_planning)/data/gt_multistory_processed.ply' ;;
  # big_maze ground truth not generated yet, volume evaluation only
  big_maze)   echo '$(rospack find motion_planning)/data/gt_big_maze_processed.ply' ;;
  *) die "unknown world '$1'";; esac; }
# Planner clearance, under half the tightest passage and over the 0.348 m f450 footprint
world_uav_radius(){ case "$1" in
  school|police) echo "1.5" ;;
  warehouse)     echo "0.8" ;;
  multistory)    echo "0.5" ;;
  big_maze)      echo "0.8" ;;
  *) die "unknown world '$1'";; esac; }
# AEP gain threshold g_zero, lower means more frontiers
world_g_zero(){ case "$1" in
  school|police|big_maze) echo "5.0" ;;
  warehouse) echo "3.0" ;;
  multistory) echo "2.0" ;;
  *) die "unknown world '$1'";; esac; }
# AEP waypoint advance distance, smaller cuts fewer corners
world_wp_reach(){ case "$1" in
  school|police) echo "0.8" ;;
  warehouse|multistory|big_maze) echo "0.5" ;;
  *) die "unknown world '$1'";; esac; }
# AEP sizing, N_max N_termination N_min_nodes radius step_size tolerance
world_aep_params(){ case "$1" in
  school|police) echo "50 300 300 3.0 2.0 1.5" ;;
  warehouse)     echo "250 1000 1000 2.0 1.5 1.0" ;;
  multistory)    echo "400 1600 1600 2.0 1.0 1.0" ;;
  big_maze)      echo "250 1000 1000 2.0 1.5 1.0" ;;
  *) die "unknown world '$1'";; esac; }
# RH_NBVP sizing, N_max N_termination radius step_size tolerance
world_nbv_params(){ case "$1" in
  school|police) echo "50 300 2.0 2.0 0.5" ;;
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
  *) die "unknown world '$1'";; esac; }

# Set the world, arg school | police | warehouse | multistory | big_maze
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

# ---- One Condition Run ----
# Run one condition, args label marginal_gain rrt_star
run_cond(){
  local label="$1" marg="$2" rrt="${3:-false}"
  local spec="${label}:${rrt}:${marg}:gpu:false:false"
  log "===== $label START (target N=$N x ${T}s, early_stop=$EARLY_STOP, spec=$spec) ====="
  if [ "$DRY" = 1 ]; then
    echo "       DRY: AEP_EARLY_STOP=$EARLY_STOP AEP_EARLY_STOP_GRACE=$GRACE \\"
    echo "            bash ./supervise_runs.sh \"$label\" \"$N\" \"$T\" \"$spec\""
    return 0
  fi
  AEP_EARLY_STOP="$EARLY_STOP" AEP_EARLY_STOP_GRACE="$GRACE" \
    bash ./supervise_runs.sh "$label" "$N" "$T" "$spec" \
      >>"$LOGDIR/run_${label}.log" 2>&1 || log "WARN supervise $label returned nonzero"
  log "===== $label DONE ====="
}

# ---- Evaluation ----
# Evaluate the campaign labels
eval_campaign(){
  local GT GTCFG L
  GT=$(world_gt "$WORLD"); GTCFG="$WORLD"
  local suffix="${KEEP_FOLDER#multi_series_}"
  log "=== EVAL stage-1 per label ($WORLD GT=$GTCFG) ==="
  if [ "$DRY" = 1 ]; then
    for L in $ALL_LABELS; do echo "       DRY: eval stage-1 $L (target_directory=data/$L gt=$GT cfg=$GTCFG)"; done
    echo "       DRY: eval stage-2 multi_series series_labels=$LABELS_CSV -> mv multi_series_evaluation -> $KEEP_FOLDER"
    return 0
  fi
  docker start noetic_ws >/dev/null 2>&1; sleep 4
  for L in $ALL_LABELS; do
    docker exec -e MPLBACKEND=Agg noetic_ws bash -lc '
      source /home/ros1/voxblox_ws/devel/setup.bash; source /home/ros1/ros1_motion_ws/devel/setup.bash
      roslaunch motion_planning full_voxblox_eval.launch \
        target_directory:=$(rospack find motion_planning)/data/'"$L"' method:=all multi_series:=false \
        evaluate:=true evaluate_volume:=false \
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
      evaluate:=true evaluate_volume:=false \
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
      for d in "$DATA/$L"/2*; do [ -d "$d" ] && [ "$(wc -l < "$d/voxblox_data.csv" 2>/dev/null||echo 0)" -gt 1 ] && n=$((n+1)); done
      echo "  $L: $n/$N good"
    done
    echo ""; echo "Plot: data/$KEEP_FOLDER/MultiSeriesOverview.png"
    echo "IF A RUN STALLS: do NOT delete it -> python3 ../../scripts/evaluation/analysis/stall_forensics.py <its tmp_bags/*.bag> $((T/60))"
  } | tee -a "$SUMMARY"
}
