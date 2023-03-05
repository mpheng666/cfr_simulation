#!/bin/bash
sudo apt install tmuxinator -y
sudo rm ~/.config/tmuxinator/*
FILES="/home/mpheng/workspace/cfr_ws/src/cfr_simulation/auto_scripts/tmux/tmuxinator/*"
for f in $FILES
do
    ln -s $f ~/.config/tmuxinator/
done