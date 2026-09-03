#!/bin/bash
# Pull the wall clock from the PC before a flight session starts.
#
# The Jetson RTC has no backup cell, so after every power cycle the clock reads
# 1970 and every run directory is stamped 19700101_*, which eval_real.sh's 2*_*
# glob never matches.  This runs ONCE, before roscore, on purpose: stepping the
# clock while nodes are up would corrupt bag and TF timestamps.
#
# Never fatal -- no PC in range just means the clock stays as it is.
set -u
PC="${PC_HOST:-lt-l4@192.168.50.2}"

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
