#!/bin/bash

# the purpose of this script is to automate setup in a way
# that keeps git potentially aware of tracker parameter changes.

# TODO will have to put data file types in gitignore?
# problems with catkin? ROS problems with the symlinks?

FROM="$(realpath examples/demo/)"
TO="$HOME/demo"

if [ ! -e $TO ]; then
printf "Making link to demo directory (original in $FROM) "\
"at $TO)\n"
ln -s $FROM $TO

else
printf "Didn't make symlink to examples/demo because "\
"~/demo exists\n"
fi

mkdir $FROM/data
