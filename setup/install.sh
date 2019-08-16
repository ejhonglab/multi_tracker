#!/usr/bin/env bash

sudo apt update
sudo apt install -y git

curl -s https://raw.githubusercontent.com/tom-f-oconnell/multi_tracker/master/setup/install_ros.sh | bash

CATKIN_SRC="$HOME/catkin/src"
if [ ! -d $CATKIN_SRC ]; then
	mkdir -p $CATKIN_SRC
fi

# TODO check for ssh key first?
# this clone syntax fine?
git clone git://github.com/tom-f-oconnell/multi_tracker.git ~/catkin/src/.

source /opt/ros/kinetic/setup.bash
rosdep install -y multi_tracker

cd ~/catkin
# TODO how to get catkin to install other required repos from source?
# separate invokation of something like wstool?
catkin_make

SRC="~/src"
if [ ! -d $SRC ]; then
	mkdir $SRC
fi

# TODO if cant use git+<url> keys for pip, package this for pypi?
git clone git://github.com/florisvb/FlyPlotLib.git ~/src/FlyPlotLib &&
(cd ~/src/FlyPlotLib && sudo -H python setup.py install)
# -H appropriate? better means of install?


