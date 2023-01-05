#!/bin/bash
session="cfr_socket_state_machine"

tmux has-session -t $session
# if [ $? != 0 ]; then
    # create new tmux session 
    tmux new-session -d -s $session

    tmux set -g mouse on

    tmux set -g status-bg black
    tmux set -g status-fg white

    window=0
    tmux rename-window -t $session:$window 'workspace'
    tmux send-keys -t $session:$window '. ~/cfr_project/cfr_ws/install/local_setup.bash; ros2 run cfr_state_machine cfr_sm_node' C-m

    tmux split-window -v
    tmux send-keys -t $session:$window '. ~/cfr_project/cfr_ws/install/local_setup.bash; sleep 2; ros2 run cfr_socket_comm cfr_socket_server_node 10000' C-m 
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window '. ~/cfr_project/cfr_ws/install/local_setup.bash; sleep 3; ros2 run cfr_socket_comm cfr_socket_client_node localhost 10000' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window '. ~/cfr_project/cfr_ws/install/local_setup.bash; sleep 3;' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window '. ~/cfr_project/cfr_ws/install/local_setup.bash; sleep 3; ros2 run cfr_feedback_server cfr_feedback_client_node localhost 10001' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window '. ~/cfr_project/cfr_ws/install/local_setup.bash; sleep 3; ros2 run cfr_feedback_server cfr_feedback_client_node localhost 10001' C-m
    tmux select-layout tiled

# fi

tmux attach-session -t $session

