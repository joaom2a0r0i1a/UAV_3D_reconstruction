#!/bin/bash
# 1 Hz host sampler: USB interrupts, load and disk writes, so a stall has a trace.
# Runs the pose watchdog in the foreground; both land in this window's tmux log.
INTERVAL="${DIAG_INTERVAL:-1}"

sampler() {
  local pi=0 pw=0
  while true; do
    local irq load wr
    irq=$(awk '/xhci|3610000/ {for(i=2;i<=NF;i++) if ($i ~ /^[0-9]+$/) s+=$i} END{print s+0}' /proc/interrupts)
    load=$(cut -d' ' -f1-3 /proc/loadavg)
    wr=$(awk '{s+=$10} END{print int(s/2)}' /proc/diskstats)   # sectors -> KB
    if [ "$pi" -ne 0 ]; then
      printf '[diag] %s  xhci_irq/s=%-7s load=%-18s disk_write=%s KB/s\n' \
        "$(date +%H:%M:%S)" "$((irq - pi))" "$load" "$((wr - pw))"
    fi
    pi=$irq; pw=$wr
    sleep "$INTERVAL"
  done
}

sampler &
SAMPLER=$!
trap 'kill $SAMPLER 2>/dev/null' INT TERM EXIT

rosrun motion_planning_real_world pose_watchdog.py
