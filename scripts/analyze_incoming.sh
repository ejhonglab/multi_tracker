#!/usr/bin/env bash

DATA_DIR=$HOME/data

while true;
do
	echo "waiting for file write"
	INCOMING_DATA_DIR=$(inotifywait -r -q --format '%w' -e close_write -- $DATA_DIR)
	echo "waiting for file writing to stop"
	while true;
	do
		inotifywait -r -q -e close_write -t 30 -- $DATA_DIR
		exit_status=$?
		echo "exit status was ${exit_status}"
		if [ $exit_status -eq 2 ]; then
			break;
		fi
	done
	echo "no writes within timeout"
	# we have reached timeout without another file being closed
	# so rsync is likely done transfering the data

	echo "waiting for ROS to shutdown"
	# wait until ROS is no longer running, checking every 10s
	while true;
	do
		rostopic list 2&>1 >/dev/null || break
		sleep 10
	done
	# TODO also run on any directories created after first incoming detected?
	# run tracking on the video, as recreated from the bag file
	rosrun multi_tracker retrack ${INCOMING_DATA_DIR}

	# -> run analysis pipeline
	# TODO have that upload all from each experiment to their own evernote thing?
	# see stackoverflow.com/questions/786376/how-do-i-run-a-program-with-a-
	# different-working-directory-from-current-from-lin
	(cd $DATA_DIR && exec rosrun choice choice_analysis)
	# else error?
done
