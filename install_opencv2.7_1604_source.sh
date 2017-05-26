#!/bin/bash

# adapted from Adrian Rosebrock's tutorial at
# pyimagesearch.com (he covers other versions as well)
# all apt commands need sudo
sudo apt update
sudo apt upgrade
sudo apt install build-essential cmake pkg-config \
libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev \
gfortran python2.7-dev python3.5-dev

SOURCE_DIR=$HOME/src
VERSION=3.2.0
 
# change this to wherever you want the source to OpenCV
cd $SOURCE_DIR
wget -O opencv.zip https://github.com/Itseez/opencv/\
archive/$VERSION.zip
unzip opencv.zip

wget -O opencv_contrib.zip https://github.com/Itseez/\
opencv_contrib/archive/$VERSION.zip
unzip opencv_contrib.zip

# TODO remove archives

# I don't plan to use virtualenvs for OpenCV
# (I want a global install)
# see Adrian's tutorial if you want to isolate the OpenCV install

# I assume numpy is already installed

cd $SOURCE_DIR/opencv-$VERSION/
mkdir build
cd build

# I modified the PYTHON_EXECUTABLE argument since I am not
# using a virtualenv
# TODO other flags i want to change?
# TODO might need to enable gstreamer (will require installing
# other libraries first)

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=$SOURCE_DIR\
/opencv_contrib-$VERSION/modules \
    -D BUILD_EXAMPLES=ON ..

#-D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \

# it is easier to troubleshoot problems without multithreaded
# compiling (remove entire -j4 flag), but it goes faster with it
make -j4

# both need sudo
sudo make install
sudo ldconfig

