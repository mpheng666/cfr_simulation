#!/bin/bash
session="cfr"

tmux has-session -t $session
if [ $? != 0 ]; then
    # create new tmux session 
    tmux new-session -d -s $session

    tmux set -g mouse on

    tmux set -g status-bg black
    tmux set -g status-fg white

    # roscore
    window=0
    tmux rename-window -t $session:$window 'workspace'
    tmux send-keys -t $session:$window 'cd ~/cfr_ws; ros2 launch cfr_gazebo gazebo.launch.py' C-m

    tmux split-window -v
    tmux send-keys -t $session:$window 'sleep 2; ros2 launch cfr_telejoy_config teleop-launch.py joy_config:=f710' C-m 
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_blades_control start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 4; ros2 launch path_tracer start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_actuation start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_dynamics start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_kinematics_states start.launch.py' C-m
    tmux select-layout tiled

fi

tmux attach-session -t $session

