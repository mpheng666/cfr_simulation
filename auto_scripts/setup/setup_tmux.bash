#!/bin/bash
sudo apt install tmuxinator -y
DIR=~/.config/tmuxinator
if [ -d "$DIR" ];
then
    # sudo rm ~/.config/tmuxinator/*
    echo "$DIR directory exists."
else
    echo "$DIR directory does not exist, creating new directory"
    mkdir "$DIR"
fi
# FILES="/home/mpheng/workspace/cfr_ws/src/cfr_simulation/auto_scripts/tmux/tmuxinator/*"
FILES=~/workspace/cfr_ws/src/auto_scripts/tmux/tmuxinator/*
for f in "$FILES"
do
    ln -s $f ~/.config/tmuxinator/
done