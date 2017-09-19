#!/usr/bin/env bash

echo "calling rosdep update"
rosdep update
echo "after rosdep update"

echo "" >> ~/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

mkdir -p ~/catkin/src
ln -s /vagrant ~/catkin/src/multi_tracker
echo "source ~/catkin/devel/setup.bash" >> ~/.bashrc

mkdir ~/src
echo "before user lines"
echo $USER
whoami
echo $(whoami)
echo "after user lines"
# TODO i can run this after provisioning fails (here) and then vagrant ssh, same command
# why doesnt it work during provisioning?
git clone git@github.com:tom-f-oconnell/rosdistro.git ~/src/rosdistro
echo "after git clone"
