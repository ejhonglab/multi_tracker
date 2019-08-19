#!/usr/bin/env bash

sudo apt update
sudo apt install -y git

curl -s https://raw.githubusercontent.com/tom-f-oconnell/multi_tracker/master/setup/install_ros.sh | bash

CATKIN_SRC="$HOME/catkin/src"
if [ ! -d $CATKIN_SRC ]; then
	mkdir -p $CATKIN_SRC
fi
cd ~/catkin
catkin_make
echo "if [ -f $HOME/catkin/devel/setup.bash ]; then" >> ~/.bashrc
echo "  source $HOME/catkin/devel/setup.bash" >> ~/.bashrc
echo "fi" >> ~/.bashrc

# TODO check for ssh key first (-> use git@... remote)?
# otherwise use https?
git clone git://github.com/tom-f-oconnell/multi_tracker.git ~/catkin/src/multi_tracker

source /opt/ros/kinetic/setup.bash
# TODO this install image_view + other necessities?
rosdep install -y multi_tracker
if [ $? -eq 0 ]; then
    echo "multi_tracker dependencies installed successfully"
else
    echo "multi_tracker dependencies were NOT installed successfully. exiting!"
    exit 1
fi

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


