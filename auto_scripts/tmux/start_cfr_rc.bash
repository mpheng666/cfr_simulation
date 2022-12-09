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
    tmux send-keys -t $session:$window 'sleep 2; ros2 launch teleop_twist_joy teleop-launch.py joy_config:=spmr1010' C-m 
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_blades_control start_rc.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 4; ros2 launch path_tracer start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch joy_publisher start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_actuation start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_dynamics start.launch.py' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3; ros2 launch cfr_states start.launch.py' C-m
    tmux select-layout tiled
fi

tmux attach-session -t $session


