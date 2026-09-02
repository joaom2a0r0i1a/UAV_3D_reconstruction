#!/bin/bash
# Stops a real flight tmux session cleanly: Ctrl+C every pane first so rosbag closes its bag and
# the recorder finalises the run, then kill the session.
# Usage: kill.sh [session]   with no argument, every session on the uav socket is stopped.

SOCKET=uav
GRACE="${KILL_GRACE:-5}"

stop_session() {
  local s="$1"
  echo "[kill] Ctrl+C to all panes of '$s'"
  tmux -L $SOCKET list-panes -s -t "$s" -F '#{pane_id}' 2>/dev/null | while read -r p; do
    tmux -L $SOCKET send-keys -t "$p" C-c
  done
  echo "[kill] waiting ${GRACE}s for the bag and the recorder to close"
  sleep "$GRACE"
  tmux -L $SOCKET kill-session -t "$s" 2>/dev/null && echo "[kill] session '$s' killed"
}

if [ -n "${1:-}" ]; then
  stop_session "$1"
else
  sessions=$(tmux -L $SOCKET ls -F '#{session_name}' 2>/dev/null)
  [ -z "$sessions" ] && { echo "[kill] no session on socket '$SOCKET'"; exit 0; }
  for s in $sessions; do stop_session "$s"; done
fi
