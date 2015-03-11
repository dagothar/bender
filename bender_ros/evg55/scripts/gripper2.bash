#!/bin/bash

# This script launches gripper_2 node and configures screen to use it with convenience

# Create a new tmux session
SESSION="Gripper2"
tmux new-session -d -s $SESSION -n "Gripper2"
tmux bind-key q confirm-before kill-session

# Prepare pane layout
tmux split-window -h
tmux split-window -v

# Launch node
tmux select-pane -t 1
tmux send-keys "clear" C-m
tmux send-keys "roslaunch ur evg55_2.launch" C-m

tmux select-pane -t 0
tmux send-keys "clear" C-m
tmux send-keys "rostopic echo /gripper_2/state" C-m

tmux select-pane -t 2
tmux send-keys "clear" C-m
tmux send-keys "echo Call services here!" C-m

# Attach to session
tmux select-window -t $SESSION:1
tmux attach-session -t $SESSION
