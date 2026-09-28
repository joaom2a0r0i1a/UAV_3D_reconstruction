#!/bin/bash
# Sourced, sets the world and spawn of $PLANNER_ENV from uav_gazebo_environments/config

ENV_CONFIG_DIR="$(rospack find uav_gazebo_environments 2>/dev/null)/config"
# uav_gazebo_environments submodule on the host
[ -d "$ENV_CONFIG_DIR" ] || ENV_CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../uav_gazebo_environments/config" && pwd)"

# Value of a key, args environment key
env_value() {
  local f="$ENV_CONFIG_DIR/$1.yaml"
  [ -f "$f" ] || {
    echo "environment.sh: no environment '$1' ($f)" >&2
    return 1
  }
  sed -n "s/^$2:[[:space:]]*//p" "$f" | sed 's/[[:space:]]*#.*//' | tr -d '[],'
}

# GainConfig.yaml environment without PLANNER_ENV
GAIN_CONFIG="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../core/gain_evaluation/config" && pwd)/GainConfig.yaml"
export PLANNER_ENV=${PLANNER_ENV:-$(sed -n 's/^environment:[[:space:]]*//p' "$GAIN_CONFIG")}
export PLANNER_WORLD_FILE="$(env_value "$PLANNER_ENV" world)"
export PLANNER_SPAWN="$(env_value "$PLANNER_ENV" spawn)"
