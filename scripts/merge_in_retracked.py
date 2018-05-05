#!/usr/bin/env python

"""
Sorts the unique output of the original acquisition (under $DATA_DIR or
$DATA_DIR/original) into the output of the offline tracking (under
subdirectories of $DATA_DIR/retracked).
"""

from __future__ import print_function

import argparse
import os
import glob
import warnings

# TODO maybe default to using stuff relative to $DATA_DIR if these aren't
# specified?
parser = argparse.ArgumentParser(description=__doc__)
# TODO add help
parser.add_argument('--from')
parser.add_argument('--to')
args = parser.parse_args()

if args.to is None:
    default_destination_ok = True
    if 'DATA_DIR' in os.environ:
        args.to = os.path.join(os.environ['DATA_DIR'], 'retracked')
        if not os.path.exists(args.to):
            default_destination_ok = False
    else:
        default_destination_ok = False

    if not default_destination_ok:
        # TODO print usage or something?
        raise NotImplementedError('must pass --to argument if DATA_DIR env' +
            'ironment variable is not set, or $DATA_DIR/retracked not a path')

for experiment_dir in retracked_experiment_directories:
    for replicate_dir in retracked_replicate_directories:
        # TODO make sure there is only one or each original dir w/ same name
        # and also that there is only one tracking output of each

        # TODO check original and retracked are on same drive (to avoid needing
        # to check for free space?) or just don't do destructive operations?

        original_data_dir = 
        # TODO deal with case where no corresponding original dir can be found

        for fname in files_in_original:
            if fname in files_in_retracked:
