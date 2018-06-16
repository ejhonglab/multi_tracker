#!/usr/bin/env bash

# TODO so the old <env> tag could seem to set this remotely, which allows
# configuration on the client side... which seems vastly preferable.
# - is there some way to do that I'm missing?
# - otherwise, maybe PR?

# Get the last host that logged in, and assume it is the ROS launch login.
# TODO filter by the actual public keys specified to be for ROS in
# authorized_keys... (seems I might need root access to get key though)
HOST=$(last -n 1 | head -n 1 | awk '{print $3}')

# Redirecting stdout to stderr, so that this can be seen on the machine
# calling roslaunch. Will only print if communication back to master fails.
printf "Determined the initiating host (assumed to be correct " 1>&2
printf "ROS_MASTER_URI) is: ${HOST}\n" 1>&2

# So far, it seems that "export" isn't necessary, nor ROS_HOSTNAME
ROS_MASTER_URI=http://${HOST}:11311

# Since this script isn't executed from its current path, but we still
# want to specify path to catkin setup.bash relative to this script.
ls ${SCRIPTPATH}/../../../devel/ 1>&2
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
. ${SCRIPTPATH}/../../../devel/setup.bash

exec "$@"
