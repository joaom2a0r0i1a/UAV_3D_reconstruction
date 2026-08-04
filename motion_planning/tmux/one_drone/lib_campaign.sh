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
#   2. sed AEPlanner.yaml  : marginal_gain, absolute_pathsum, bounded_box min_x/max_x (world)
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
# AEP bounded_box min_x/max_x per world (y/z unchanged across worlds, as in originals)
world_aep_box(){ case "$1" in
  school) echo "-25.0 25.0" ;;
  police) echo "-15.0 15.0" ;;
  *) die "unknown world '$1'";; esac; }
# GainConfig gain_evaluation region: min_x max_x min_y max_y min_z max_z
world_gain_region(){ case "$1" in
  school) echo "-17.0 17.0 -12.0 7.0 0.0 14.5" ;;
  police) echo "-7.0 7.0 -8.5 8.5 0.0 9.0" ;;
  *) die "unknown world '$1'";; esac; }
world_world_file(){ case "$1" in
  school) echo "grass_plane_school.world" ;;
  police) echo "grass_plane_police_station.world" ;;
  *) die "unknown world '$1'";; esac; }
world_spawn_pos(){ case "$1" in
  school) echo "-22 0 0.5 0" ;;
  police) echo "-12 0 0.5 0" ;;
  *) die "unknown world '$1'";; esac; }
world_marker(){ case "$1" in
  school) echo "School" ;;
  police) echo "Police Station" ;;
  *) die "unknown world '$1'";; esac; }
world_gt(){ case "$1" in
  school) echo '$(rospack find motion_planning)/data/gt_school_processed.ply' ;;
  police) echo '$(rospack find motion_planning)/data/gt_police_station_processed.ply' ;;
  *) die "unknown world '$1'";; esac; }
world_gtcfg(){ case "$1" in
  school) echo "School.yaml" ;;
  police) echo "PoliceStation.yaml" ;;
  *) die "unknown world '$1'";; esac; }

# ---- a single value-replace helper (key-based; hits only the active/uncommented key)
set_key(){  # $1=file $2=key $3=value  (value regex covers numbers, true/false)
  sed -i -E "s@^([[:space:]]*$2:)[[:space:]]*[-0-9a-z.]+@\1 $3@" "$1"
}

# ---- world switch: session.yml world+spawn, AEP box, GainConfig region ------
set_world(){  # $1=school|police
  local w="$1" other wf pos box gr marker
  [ "$w" = school ] && other=police || other=school
  wf=$(world_world_file "$w");  pos=$(world_spawn_pos "$w")
  # session.yml: uncomment target world/spawn, comment the other
  local wf_other=$(world_world_file "$other")  pos_other=$(world_spawn_pos "$other")
  sed -i -E "s@^([[:space:]]*)#-([[:space:]]*waitForRos.*${wf//./\\.}.*)@\1-\2@"        "$SESS"
  sed -i -E "s@^([[:space:]]*)-([[:space:]]*waitForRos.*${wf_other//./\\.}.*)@\1#-\2@"  "$SESS"
  sed -i -E "s@^([[:space:]]*)#-([[:space:]]*waitForGazebo.*--pos ${pos} .*)@\1-\2@"        "$SESS"
  sed -i -E "s@^([[:space:]]*)-([[:space:]]*waitForGazebo.*--pos ${pos_other} .*)@\1#-\2@" "$SESS"
  # AEPlanner.yaml bounded_box min_x/max_x
  read -r bmin bmax <<< "$(world_aep_box "$w")"
  set_key "$YAML" min_x "$bmin"; set_key "$YAML" max_x "$bmax"
  # GainConfig gain_evaluation region + active-world marker
  read -r gmnx gmxx gmny gmxy gmnz gmxz <<< "$(world_gain_region "$w")"
  set_key "$GCFG" min_x "$gmnx"; set_key "$GCFG" max_x "$gmxx"
  set_key "$GCFG" min_y "$gmny"; set_key "$GCFG" max_y "$gmxy"
  set_key "$GCFG" min_z "$gmnz"; set_key "$GCFG" max_z "$gmxz"
  marker=$(world_marker "$w")
  sed -i -E "s@^gain_evaluation:.*@gain_evaluation:  # ${marker} (active)@" "$GCFG"
  log "world -> $w (session world+spawn, AEP box=${bmin}/${bmax}, gain region + marker)"
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
set_nbvp(){  # $1=optyaw $2=nmax $3=nterm $4=step $5=fixed
  set_key "$NYAML" optimize_yaw   "$1"
  set_key "$NYAML" N_max          "$2"
  set_key "$NYAML" N_termination  "$3"
  set_key "$NYAML" step_size      "$4"
  set_key "$NYAML" fixed_step     "$5"
  [ "$3" -gt "$2" ] || die "N_termination ($3) MUST be > N_max ($2) or receding-horizon never recedes"
  log "NBVP optimize_yaw=$1 N_max=$2 N_term=$3 step=$4 fixed_step=$5"
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
