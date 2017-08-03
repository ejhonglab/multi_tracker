#!/bin/bash

# will likely need to run this script with at least 
# sudo privileges
if [[ $(id -u -n) == "root" ]]; then

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-get update
apt-get install ros-kinetic-desktop-full

rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

apt-get install python-rosinstall

# idk which line caused this to be needed. does it work here?
rosdep fix-permissions

touch $DIR.ros_installed

else
echo "Either run this script as root or use sudo."
fi
