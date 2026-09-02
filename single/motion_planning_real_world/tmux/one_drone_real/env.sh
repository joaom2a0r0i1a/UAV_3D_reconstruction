# Sourced by every window of the real flight tmux sessions. Edit here, not in the session files.

# ---- experiment identity -------------------------------------------------------------
# PLANNER is set by the session script (rhnbvp, aep, kino-aep, kino-nbvp).
# GAIN is the variant under test and only LABELS the run, see the warning below.
export PLANNER="${PLANNER:-rhnbvp}"
export GAIN="${GAIN:-marginal}"                 # marginal | absolute
export RUN_LABEL="${RUN_LABEL:-${PLANNER}_${GAIN}}"

# GAIN drives the planner too: the session passes marginal_gain:=$MARGINAL to the planner
# launch, which overrides evaluation/marginal_gain from the yaml. So the label cannot disagree
# with the run. Each run also stores the resolved parameters in rosparams.yaml.
case "$GAIN" in
  marginal) export MARGINAL=true ;;
  absolute) export MARGINAL=false ;;
  *) echo "env.sh: GAIN must be marginal or absolute, got '$GAIN' - falling back to the yaml" >&2
     export MARGINAL=yaml ;;
esac

# ---- where runs land -----------------------------------------------------------------
# EXP_ROOT is what eval_real.sh is pointed at; EXP_DIR is one variant inside it.
export EXP_ROOT="${EXP_ROOT:-$HOME/real_experiments}"
export EXP_DIR="${EXP_DIR:-$EXP_ROOT/$RUN_LABEL}"

# ---- ROS + MAVLink identities --------------------------------------------------------
# ROS namespace for the planner, recorder and voxblox. Unrelated to the MAVLink ids below.
export UAV_NAME="${UAV_NAME:-uav1}"

# MAVLink identity of the autopilot, its SYSID_THISMAV parameter. mavros addresses every
# command to this id, so a mismatch means the FCU silently drops all setpoints.
export FCU_SYSID="${FCU_SYSID:-2}"

# Serial link to the autopilot. SERIAL2 is MAVLink2 at 921600, SERIAL1 at 57600.
export FCU_URL="${FCU_URL:-/dev/ttyUSB0:921600}"
export FCU_DEV="${FCU_URL%%:*}"

# ---- helpers -------------------------------------------------------------------------
# Stand-ins for the MRS helpers: those wait on a control stack this setup does not run.
waitForRos() { until rostopic list > /dev/null 2>&1; do sleep 1; done; }
waitForMavros() { waitForRos; until rostopic list 2>/dev/null | grep -q '^/mavros/state$'; do sleep 1; done; }
