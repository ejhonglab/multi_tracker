#!/usr/bin/env bash

DATA_DIR=$HOME/data

while true;
do
    inotifywait -r -e close_write $DATA_DIR
    inotifywait -r -e close_write -t 30
    exit_status=$?
    if [ $exit_status -eq 0 ]; then
        continue
    elif [ $exit_status -eq 2 ]; then
	# we have reached timeout without another file being closed
	# so rsync is likely done transfering the data

	# wait until ROS is no longer running, checking every 10s
	while true;
	do
	    rostopic list 2&>1 >/dev/null || break
	    sleep 10
	done

	# TODO how to get which directories are new? or have python analysis walk all?
	# just run on any newer than last time this was run, and store that info in
	# a file in data dir?

	# run tracking on the video, as recreated from the bag file
	ROS_HOME=

	# -> run analysis pipeline
	# TODO have that upload all from each experiment to their own evernote thing?
    fi
    # else error?
done
