#!/usr/bin/env bash

sudo apt update
sudo apt install git

./install_ros.sh

mkdir -p ~/catkin/src
# TODO check for key first?
git clone git://github.com/tom-f-oconnell/multi_tracker.git ~/catkin/src/.

cd ~/catkin
catkin_make
