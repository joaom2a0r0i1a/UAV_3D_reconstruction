#!/bin/bash
# =============================================================================
# lib_campaign.sh — shared library for run_campaign.sh (sourced, not executed).
#
# Consolidates the config-injection + run + eval backbone that the ~25 one-off
# experiment drivers (school_*, police_*, nbvp_*, aep_*) all duplicated. Those
# drivers differed ONLY in a config descriptor (world, planner, gain mode,
# NBVP N_max/step/fixed_step, N, T, early-stop, labels);
# everything below is byte-for-byte what they shared.
#
# Injection channels (unchanged from the originals):
#   1. config-spec -> current_config.env -> session.yml -> planner .launch optenv
#      "label:rrt_star:marginal_gain:compute:marginal_split:benchmark"
#   2. sed AEPlanner.yaml  : marginal_gain, absolute_pathsum, bounded_box (full 6-dim, per world)
#   3. sed NBVPlanner.yaml : optimize_yaw, N_max, N_termination, step_size, fixed_step
#   4. sed GainConfig.yaml : gain_evaluation region (world) + active-world marker
#   5. sed session.yml     : world+spawn lines, AEPlanner<->NBVPlanner launch line
#
# File paths (YAML/NYAML/GCFG/SESS) are set by run_campaign.sh so the same
# helpers can operate on temp copies during --dry-run.
# =============================================================================

log(){ echo "[$(date +%H:%M:%S)] $*"; }
die(){ echo "[$(date +%H:%M:%S)] FATAL: $*" >&2; exit 1; }

# ---- world region tables (single source of truth) --------------------------
ALL_WORLDS="school police warehouse multistory big_maze"
# AEP bounded_box (RRT sampling region): min_x max_x min_y max_y min_z max_z
world_aep_box(){ case "$1" in
  school)     echo "-25.0 25.0 -20.0 15.0 1.0 20.0" ;;
  police)     echo "-15.0 15.0 -20.0 15.0 1.0 20.0" ;;
  warehouse)  echo "-20.0 20.0 -12.0 12.0 0.5 7.0" ;;
  multistory) echo "-10.0 10.0 -8.0 8.0 0.5 9.8" ;;
  big_maze)   echo "-20.0 20.0 -20.0 20.0 0.5 1.8" ;;   # z-max 1.8 = 0.7m below 2.5m wall tops so the drone stays INSIDE the maze (not over it)
  *) die "unknown world '$1'";; esac; }
# GainConfig gain_evaluation region: min_x max_x min_y max_y min_z max_z
world_gain_region(){ case "$1" in
  school)     echo "-17.0 17.0 -12.0 7.0 0.0 14.5" ;;
  police)     echo "-7.0 7.0 -8.5 8.5 0.0 9.0" ;;
  warehouse)  echo "-20.0 20.0 -12.0 12.0 0.0 7.0" ;;
  multistory) echo "-10.0 10.0 -8.0 8.0 0.0 9.9" ;;
  big_maze)   echo "-20.0 20.0 -20.0 20.0 0.0 2.5" ;;
  *) die "unknown world '$1'";; esac; }
world_world_file(){ case "$1" in
  school)     echo "grass_plane_school.world" ;;
  police)     echo "grass_plane_police_station.world" ;;
  warehouse)  echo "grass_plane_warehouse.world" ;;
  multistory) echo "grass_plane_multistory.world" ;;
  big_maze)   echo "big_maze.world" ;;
  *) die "unknown world '$1'";; esac; }
world_spawn_pos(){ case "$1" in
  school)     echo "-22 0 0.5 0" ;;
  police)     echo "-12 0 0.5 0" ;;
  warehouse)  echo "-18 0 1.0 0" ;;       # inside west staging by the man-door, facing the racks
  multistory) echo "0 0 1.0 1.57" ;;      # atrium centre (open shaft), facing +y
  big_maze)   echo "0 0 0.5 0" ;;           # maze centre (3.13m clearance), origin frame
  *) die "unknown world '$1'";; esac; }
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
  big_maze)   echo '$(rospack find motion_planning)/data/gt_big_maze_processed.ply' ;;   # NOT YET GENERATED - volume-mode eval only
  *) die "unknown world '$1'";; esac; }
world_gtcfg(){ case "$1" in
  school)     echo "School.yaml" ;;
  police)     echo "PoliceStation.yaml" ;;
  warehouse)  echo "Warehouse.yaml" ;;
  multistory) echo "MultiStory.yaml" ;;
  big_maze)   echo "BigMaze.yaml" ;;
  *) die "unknown world '$1'";; esac; }
# Collision-avoidance clearance the planner demands from every node AND edge. MUST be < half the
# tightest passage (multistory corridor 1.5 m, warehouse aisle 2.4 m) or the space is unflyable, and
# >= the physical f450 footprint = max(center->prop-tip) = 0.230 m arm + 0.118 m blade = 0.348 m.
# All values below are max(chosen, 0.348) so the drone body always fits inside the cleared corridor.
world_uav_radius(){ case "$1" in
  school|police) echo "1.5" ;;
  warehouse)     echo "0.8" ;;
  multistory)    echo "0.5" ;;  # highest flyable in the 1.5 m corridors (0.8 would leave no valid corridor window)
  big_maze)      echo "0.8" ;;  # corridors 2.2-3.9 m (min 2.23) -> 0.8 leaves a 0.6 m valid tube
  *) die "unknown world '$1'";; esac; }
# AEP gain threshold (g_zero): a node/frontier counts as worth visiting when gain > g_zero. Lower = more
# frontiers, later termination, less missed reconstruction in clutter. AEP-only (NBV uses best_score==0).
world_g_zero(){ case "$1" in
  school|police|big_maze) echo "5.0" ;;
  warehouse) echo "3.0" ;;    # 2026-08-18 user set 3.0 (5.0 terminated too early / left clutter unseen; 2.0 washed out)
  multistory) echo "2.0" ;;   # tightest corridors/layout -> keep low to catch the last ~5%
  *) die "unknown world '$1'";; esac; }
# Waypoint-advance distance: publish the next waypoint when within this dist of the current one. Larger = smoother
# but cuts corners (clips tight doors on long global paths); smaller = tighter follow. AEP-only (NBV flies single targets).
world_wp_reach(){ case "$1" in
  school|police) echo "0.8" ;;
  warehouse|multistory|big_maze) echo "0.5" ;;   # tight doors -> less corner-cutting
  *) die "unknown world '$1'";; esac; }
# AEP local/global sizing + motion: N_max N_termination N_min_nodes radius step_size tolerance
# (cluttered worlds -> more nodes, higher term cap for extra collision rejections, smaller step)
world_aep_params(){ case "$1" in
  school|police) echo "50 300 300 3.0 2.0 1.5" ;;
  warehouse)     echo "250 1000 1000 2.0 1.5 1.0" ;;   # matches big_maze (2026-08-16); only uav_radius differs
  multistory)    echo "400 1600 1600 2.0 1.0 1.0" ;;
  big_maze)      echo "250 1000 1000 2.0 1.5 1.0" ;;   # node counts + step 1.5 == warehouse now
  *) die "unknown world '$1'";; esac; }
# NBV (receding-horizon) sizing + motion: N_max N_termination radius step_size tolerance
world_nbv_params(){ case "$1" in
  school|police) echo "50 300 2.0 2.0 0.5" ;;
  warehouse)     echo "250 1000 2.0 1.5 1.0" ;;   # matches big_maze/AEP (250/1000, step 1.5)
  multistory)    echo "400 1600 2.0 1.0 1.0" ;;
  big_maze)      echo "250 1000 2.0 1.5 1.0" ;;   # MUST match AEP (250/1000, step 1.5) for fair cross-planner compare
  *) die "unknown world '$1'";; esac; }
# Per-world exploration time budget in MINUTES (EXP_TIME_LIMIT / campaign T). Single source of truth;
# run_campaign derives T=<mins>*60 from this when a campaign .conf does not set T explicitly.
world_time_limit(){ case "$1" in
  school)     echo "30" ;;
  police)     echo "30" ;;
  warehouse)  echo "45" ;;
  multistory) echo "35" ;;   # AEP self-terminates ~20min; 35 gives RH-NBVP tail room + caps non-terminating runs
  big_maze)   echo "45" ;;   # CALIBRATE with a test run (large 40x40 flat maze)
  *) die "unknown world '$1'";; esac; }

# ---- a single value-replace helper (key-based; hits only the active/uncommented key)
set_key(){  # $1=file $2=key $3=value  (value regex covers numbers, true/false, words w/ _ and quotes)
  sed -i -E "s@^([[:space:]]*$2:)[[:space:]]*[-0-9a-zA-Z._\"]+@\1 $3@" "$1"
}

# ---- world switch: session.yml world+spawn, AEP box, GainConfig region ------
set_world(){  # $1=school|police|warehouse|multistory
  local w="$1" wf pos marker ow owf opos
  wf=$(world_world_file "$w"); pos=$(world_spawn_pos "$w")
  # session.yml: comment EVERY known world+spawn line, then uncomment only the target's
  for ow in $ALL_WORLDS; do
    owf=$(world_world_file "$ow"); opos=$(world_spawn_pos "$ow")
    sed -i -E "s@^([[:space:]]*)-([[:space:]]*waitForRos.*${owf//./\\.}.*)@\1#-\2@"     "$SESS"
    sed -i -E "s@^([[:space:]]*)-([[:space:]]*waitForGazebo.*--pos ${opos} .*)@\1#-\2@" "$SESS"
  done
  sed -i -E "s@^([[:space:]]*)#-([[:space:]]*waitForRos.*${wf//./\\.}.*)@\1-\2@"      "$SESS"
  sed -i -E "s@^([[:space:]]*)#-([[:space:]]*waitForGazebo.*--pos ${pos} .*)@\1-\2@"  "$SESS"
  # AEPlanner.yaml bounded_box (full 6-dim RRT sampling region)
  read -r axmn axmx aymn aymx azmn azmx <<< "$(world_aep_box "$w")"
  set_key "$YAML" min_x "$axmn"; set_key "$YAML" max_x "$axmx"
  set_key "$YAML" min_y "$aymn"; set_key "$YAML" max_y "$aymx"
  set_key "$YAML" min_z "$azmn"; set_key "$YAML" max_z "$azmx"
  # NBVPlanner.yaml bounded_box MUST match AEP's (same world, same sampling region)
  set_key "$NYAML" min_x "$axmn"; set_key "$NYAML" max_x "$axmx"
  set_key "$NYAML" min_y "$aymn"; set_key "$NYAML" max_y "$aymx"
  set_key "$NYAML" min_z "$azmn"; set_key "$NYAML" max_z "$azmx"
  # GainConfig gain_evaluation region + active-world marker
  read -r gmnx gmxx gmny gmxy gmnz gmxz <<< "$(world_gain_region "$w")"
  set_key "$GCFG" min_x "$gmnx"; set_key "$GCFG" max_x "$gmxx"
  set_key "$GCFG" min_y "$gmny"; set_key "$GCFG" max_y "$gmxy"
  set_key "$GCFG" min_z "$gmnz"; set_key "$GCFG" max_z "$gmxz"
  marker=$(world_marker "$w")
  sed -i -E "s@^gain_evaluation:.*@gain_evaluation:  # ${marker} (active)@" "$GCFG"
  # uav_radius (both planner yamls) + per-world sizing/motion params
  local uavr; uavr=$(world_uav_radius "$w")
  set_key "$YAML" uav_radius "$uavr"; set_key "$NYAML" uav_radius "$uavr"
  local gz; gz=$(world_g_zero "$w")
  set_key "$YAML" g_zero "$gz"                     # AEP gain threshold (per-world; lower in clutter)
  set_key "$CCFG" g_zero "$gz"                     # cache_nodes frontier server MUST use the same threshold
  set_key "$YAML" waypoint_reach_distance "$(world_wp_reach "$w")"   # per-world corner-cutting control
  local an at anm ar astep atol
  read -r an at anm ar astep atol <<< "$(world_aep_params "$w")"
  set_key "$YAML" N_max "$an"; set_key "$YAML" N_termination "$at"; set_key "$YAML" N_min_nodes "$anm"
  set_key "$YAML" radius "$ar"; set_key "$YAML" step_size "$astep"; set_key "$YAML" tolerance "$atol"
  local nn nt nr nstep ntol
  read -r nn nt nr nstep ntol <<< "$(world_nbv_params "$w")"
  set_key "$NYAML" N_max "$nn"; set_key "$NYAML" N_termination "$nt"      # set_nbvp may override for NBV sweeps
  set_key "$NYAML" radius "$nr"; set_key "$NYAML" step_size "$nstep"; set_key "$NYAML" tolerance "$ntol"
  log "world -> $w (session world+spawn, AEP box=${axmn}/${axmx}/${aymn}/${aymx}/${azmn}/${azmx}, gain region + marker, uav_radius=${uavr}, AEP N=${an}/${at}/${anm}, NBV N=${nn}/${nt})"
}

# ---- planner switch: session.yml launch line -------------------------------
set_planner(){  # $1=aep|nbvp
  local want other
  case "$1" in
    aep)  want=AEPlanner;  other=NBVPlanner ;;
    nbvp) want=NBVPlanner; other=AEPlanner ;;
    *) die "unknown planner '$1'";;
  esac
  sed -i -E "s@^([[:space:]]*)#-([[:space:]]*waitForControl; roslaunch motion_planning ${want}\.launch)@\1-\2@" "$SESS"
  sed -i -E "s@^([[:space:]]*)-([[:space:]]*waitForControl; roslaunch motion_planning ${other}\.launch)@\1#-\2@" "$SESS"
  log "planner -> $1 (${want}.launch)"
}

# ---- AEP gain flags --------------------------------------------------------
set_aep_gain(){  # $1=abs|marg  (legacy "control" == abs; absolute is always the path-union baseline now)
  case "$1" in
    abs|control) set_key "$YAML" marginal_gain false ;;
    marg)        set_key "$YAML" marginal_gain true ;;
    *) die "unknown gain mode '$1' (use abs|marg)";;
  esac
  log "AEP gain=$1"
  grep -E "marginal_gain:" "$YAML" | sed 's/^/       /'
}

# ---- NBVP knobs ------------------------------------------------------------
set_nbvp(){  # $1=optyaw $2=nmax $3=nterm $4=step $5=fixed $6=objective(expdecay|rate_L) $7=execution_horizon
  set_key "$NYAML" optimize_yaw   "$1"
  set_key "$NYAML" N_max          "$2"
  set_key "$NYAML" N_termination  "$3"
  set_key "$NYAML" step_size      "$4"
  set_key "$NYAML" fixed_step     "$5"
  set_key "$NYAML" objective      "${6:-expdecay}"
  set_key "$NYAML" execution_horizon "${7:-1}"
  [ "$3" -gt "$2" ] || die "N_termination ($3) MUST be > N_max ($2) or receding-horizon never recedes"
  log "NBVP optimize_yaw=$1 N_max=$2 N_term=$3 step=$4 fixed_step=$5 objective=${6:-expdecay} horizon=${7:-1}"
}

# ---- preflight: read-only asserts on the live config (catches wrong world/planner)
preflight(){  # $1=world $2=planner
  local w="$1" p="$2" wf pos marker launch
  wf=$(world_world_file "$w"); pos=$(world_spawn_pos "$w"); marker=$(world_marker "$w")
  grep -qE "^[[:space:]]*-[[:space:]]*waitForRos.*${wf//./\\.}" "$SESS"      || die "session world not $w"
  grep -qE "^[[:space:]]*-[[:space:]]*waitForGazebo.*--pos ${pos} " "$SESS"  || die "session spawn not $w"
  grep -qE "^gain_evaluation:.*${marker}" "$GCFG"                            || die "GainConfig not $w"
  case "$p" in aep) launch=AEPlanner;; nbvp) launch=NBVPlanner;; esac
  grep -qE "^[[:space:]]*-[[:space:]]*waitForControl; roslaunch motion_planning ${launch}\.launch" "$SESS" || die "planner not $p"
  log "PREFLIGHT OK — $w / $p"
}

# ---- one condition run (host-side supervisor, container-drop resilient) -----
run_cond(){  # $1=label $2=marginal_gain_spec(true|false) $3=rrt_star(false|true)
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

# ---- eval: stage-1 per label + stage-2 multi_series -> keep-folder ----------
eval_campaign(){  # uses: WORLD, ALL_LABELS, LABELS_CSV, KEEP_FOLDER, SUMMARY
  local GT GTCFG L
  GT=$(world_gt "$WORLD"); GTCFG=$(world_gtcfg "$WORLD")
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
        evaluate:=false evaluate_volume:=true \
        gt_file_path:='"$GT"' experiment_config:='"$GTCFG"'' \
      >"$LOGDIR/stage1_${L}_${suffix}.log" 2>&1
    log "  stage-1 $L done"
  done
  log "=== EVAL stage-2 multi_series ($WORLD GT) ==="
  docker exec -e MPLBACKEND=Agg noetic_ws bash -lc '
    source /home/ros1/voxblox_ws/devel/setup.bash; source /home/ros1/ros1_motion_ws/devel/setup.bash
    roslaunch motion_planning full_voxblox_eval.launch \
      target_directory:=$(rospack find motion_planning)/data \
      multi_series:=true series_labels:="'"$LABELS_CSV"'" \
      evaluate:=false evaluate_volume:=true \
      gt_file_path:='"$GT"' experiment_config:='"$GTCFG"'' \
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
    echo "IF A RUN STALLS: do NOT delete it -> python3 ../../scripts/eval/stall_forensics.py <its tmp_bags/*.bag> $((T/60))"
  } | tee -a "$SUMMARY"
}
