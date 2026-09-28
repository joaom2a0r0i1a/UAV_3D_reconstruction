#!/bin/bash
# Clock sync from the PC before roscore, never fatal
# The Jetson RTC has no backup cell
set -u
PC="${PC_HOST:-<user>@<pc-ip>}"

T=$(ssh -n -o ConnectTimeout=5 -o BatchMode=yes "$PC" 'date -u +"%Y-%m-%d %H:%M:%S"' 2>/dev/null)
if [ -z "$T" ]; then
  echo "[time] $PC unreachable -- keeping current clock $(date -Is)" >&2
  exit 0
fi

BEFORE=$(date -Is)
if sudo -n date -u -s "$T" >/dev/null 2>&1; then
  echo "[time] $BEFORE -> $(date -Is)  (from $PC)"
else
  echo "[time] passwordless 'date' unavailable -- clock left at $BEFORE" >&2
  echo "[time] run once: sudo bash /tmp/uav_time_setup.sh" >&2
fi
exit 0
