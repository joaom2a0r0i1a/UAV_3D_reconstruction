#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=mrs

NUM_RUNS=5
#SIM_TIME=2600  # seconds
SIM_TIME=1850  # seconds
CHECK_INTERVAL=30  # seconds between checks

ROS_LAUNCH_FILE="motion_planning full_voxblox_eval.launch"

function terminate_sim() {
  echo ""
  echo ">>> Killing tmux session ($TMUX_SESSION_NAME)..."

  tmux -L $TMUX_SOCKET_NAME split-window -t $TMUX_SESSION_NAME
  tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME "sleep 1; pwd >> /tmp/tmux_restore_path.txt; tmux list-panes -s -F \"#{pane_pid} #{pane_current_command}\" | grep -v tmux | cut -d\" \" -f1 | while read in; do killProcessRecursive \$in; done; exit" ENTER
  sleep 2

  echo ">>> Simulation stopped."
}

# Ensure cleanup happens on Ctrl+C
trap terminate_sim SIGINT

echo "To emergency stop the simulation inside tmux, press Ctrl+a then k then 9."
echo "To stop the whole script: press Ctrl+C"
echo "This will kill the pane and stop the script loop."

for i in $(seq 1 $NUM_RUNS); do
  echo "Starting simulation run #$i"

  # Start the tmuxinator session
  tmuxinator start -p ./session.yml

  # Attach to tmux (like original start.sh)
  #if [ -z "$TMUX" ]; then
  #  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME
  #else
  #  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME"
  #fi

  elapsed=0
  while [ $elapsed -lt $SIM_TIME ]; do
    tmux -L $TMUX_SOCKET_NAME has-session -t $TMUX_SESSION_NAME 2>/dev/null
    if [ $? -ne 0 ]; then
      echo "Tmux session disappeared! Assuming emergency stop. Exiting loop."
      terminate_sim
      exit 1
    fi

    sleep $CHECK_INTERVAL
    elapsed=$((elapsed + CHECK_INTERVAL))
  done

  echo "Terminating simulation run #$i"
  terminate_sim
  sleep 5
done

echo "All runs completed or stopped early. Launching ROS file..."
roslaunch $ROS_LAUNCH_FILE
echo "ROS launch finished. Exiting script."