# Sourced by every window of the real flight sessions

# ---- Experiment Identity ----
# PLANNER from the session script, GAIN labels the run
export PLANNER="${PLANNER:-rhnbvp}"
export GAIN="${GAIN:-marginal}"
export RUN_LABEL="${RUN_LABEL:-${PLANNER}_${GAIN}}"

# GAIN also sets marginal_gain for the planner
case "$GAIN" in
  marginal) export MARGINAL=true ;;
  absolute) export MARGINAL=false ;;
  *)
    echo "env.sh: GAIN must be marginal or absolute, got '$GAIN' - falling back to the yaml" >&2
    export MARGINAL=yaml
    ;;
esac

# ---- Run Directories ----
# EXP_ROOT for eval_rw.sh, EXP_DIR per variant
export EXP_ROOT="${EXP_ROOT:-$HOME/real_experiments}"
export EXP_DIR="${EXP_DIR:-$EXP_ROOT/$RUN_LABEL}"

# Record profile, eval | eval-viz | eval-camera | mapping-replay | full-debug
export RECORD_PROFILE="${RECORD_PROFILE:-eval-camera}"

# ---- ROS and MAVLink Identities ----
# ROS namespace, unrelated to the MAVLink ids
export UAV_NAME="${UAV_NAME:-uav1}"

# Autopilot SYSID_THISMAV, a mismatch drops all setpoints
export FCU_SYSID="${FCU_SYSID:-2}"
# start_gate starts the planner once armed and settled
export AUTO_START="${AUTO_START:-true}"

# Autopilot serial link, SERIAL2 at 921600, SERIAL1 at 57600
export FCU_URL="${FCU_URL:-/dev/ttyUSB0:921600}"
export FCU_DEV="${FCU_URL%%:*}"

# ---- Helpers ----
# Stand-ins for the MRS wait helpers
waitForRos() { until rostopic list >/dev/null 2>&1; do sleep 1; done; }
waitForMavros() {
  waitForRos
  until rostopic list 2>/dev/null | grep -q '^/mavros/state$'; do sleep 1; done
}
