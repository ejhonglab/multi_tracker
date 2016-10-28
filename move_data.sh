#!/bin/bash

if [[ $1 -eq 0 ]] ; then
    echo 'Expecting the new directory name as an argument.'
    exit 0
fi

DELTA_LAUNCH=~/src/catkin/src/multi_tracker/examples/sample_data/launch_delta_video.launch
CURDIR=grep 'arg name="path"' $DELTA_LAUNCH | grep -Po '[0-9]{2}'

mv ~/demo/demo_1/data ~/demo/demo_1/$CURDIR
mkdir ~/demo/demo_1/data

sed -i -e "/arg name=\"path\"/ s/$CURDIR/$1/" $DELTA_LAUNCH
