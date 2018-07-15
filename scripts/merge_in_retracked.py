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
import re
import warnings

# TODO option to delete from source after verifying correct copy

# TODO TODO move to util library (haven't I already wrote something doing part
# of this?)
def valid_replicate_dir(d, source=True):
    """Returns True if d is a valid replicate dir, False otherwise.

    All valid replicate directories have:
    - a name as YYYYMMDD_HHMMSS

    If source is True, it must contain at least:
    - one *_*_parameters.yaml
    - *_stimuli.p
    - a *_delta_video.bag

    If source is False, it must contain at least:
    - two *_**_parameters.yaml files, one from acquisition and one from tracking
    - one or more *_N<N>_trackedobjects.hdf5
    - one or more roi_N<N>.yaml files
    - one or more *_deltavideo_bgimg_**_N<N>.png
    """
    # TODO option(s) to fail / just print warning / ignore entirely, if required
    # files are missing 
    # TODO also check all values with restricted ranges are valid? able to use
    # something like datetime for that?
    name = os.path.basename(os.path.normpath(d))
    if not re.match('^[0-9]{8}_[0-9]{6}$', name):
        return False

    #src_required_regexes = {'.*_parameters.yaml$', '.'}

    # TODO need to do a normpath on these?
    num_param_yamls = 0
    have_stimuli = False
    have_deltavid = False
    # TODO how to handle roi,traj,png correspondence in not source case?
    # cases where i don't want to require this 1:1 correspondence?
    roi_num2have = dict()

    for e in os.scandir(d):
        # TODO if i'm not going to require stimuli file, maybe only require on
        # of these? (just kinda want to make sure it is actually the result of
        # the tracking, particularly if there is only one, and i'm just going to
        # blindly copy it to the source and consider that a success.) maybe not
        # important...
        if e.name.endswith('_parameters.yaml'):
            num_param_yamls += 1

        elif source:
            if e.name.endswith('_delta_video.bag'):
                have_deltavid = True

            elif e.endswith('_stimuli.p'):
                have_stimuli = True

        else:
            trajectory_suffix = '_trackedobjects.hdf5'
            roi_prefix = 'roi_N'
            if e.name.endswith(trajectory_suffix):
                # TODO - 1?
                n = int(e.name[(-1) * len(trajectory_suffix)])

                if n in roi_num2have:
                    roi_num2have[n].add('tracking')
                else:
                    roi_num2have[n] = set()

            elif e.name.startswith(roi_prefix) and e.name.endswith('.yaml'):
                n = int(e.name[len(roi_prefix)])

                if n in roi_num2have:
                    roi_num2have[n].add('roi')
                else:
                    roi_num2have[n] = set()

            elif '_deltavideo_bgimg_' in e.name and e.name.endswith('.png'):
                n = int(e.name[-5])

                if n in roi_num2have:
                    roi_num2have[n].add('background')
                else:
                    roi_num2have[n] = set()

            else:
                # print warning w/ file name?
                check_not_symlink = False
                pass

        if e.is_symlink():
            raise IOError('One of the required files in ' +
                '{}, {} was a symlink'.format(d, e.name))

    if source:
        if num_param_yamls < 1:
            raise IOError('Found no *_parameter.yaml files in directory ' +
                '{} containing original acquisition data.'.format(d))

        elif num_param_yamls > 1:
            raise IOError('Found more than one *_parameter.yaml files in ' =
                'directory {} containing original acquisition data.'.format(d))

        if not have_deltavid:
            raise IOError('Source directory {} was missing delta video bagfile')

        if not have_stimuli:

        return True

    else:
        if num_param_yamls < 2:
            raise IOError('Expecting at least 2 *_parameter.yaml files in ' +
                'directory {} containing data from offline tracking'.format(d))

        elif num_param_yamls > 2:
            warnings.warn('Found more than 2 *_parameter.yaml files in ' +
                'directory {} containing data from offline '.format(d) +
                'tracking. Expected 2.')

        if 

    # TODO warn if file sizes are suspiciously small?

    # TODO at least warn if requirements are not regular files (though should
    # not be the case for any of my current directories)

    
    

# TODO maybe default to using stuff relative to $DATA_DIR if these aren't
# specified?
parser = argparse.ArgumentParser(description=__doc__)
# TODO add help
parser.add_argument('--src')
parser.add_argument('--dst')
args = parser.parse_args()

if args.src is None:
    default_source_ok = True
    if 'DATA_DIR' in os.environ:
        args.src = os.environ['DATA_DIR']
        if not os.path.exists(args.src):
            default_source_ok = False
    else:
        default_source_ok = False

    if not default_source_ok:
        # TODO print usage or something?
        raise NotImplementedError('must pass --src argument if DATA_DIR env' +
            'ironment variable is not set, or $DATA_DIR is not a path')

if args.dst is None:
    default_destination_ok = True
    if 'DATA_DIR' in os.environ:
        args.dst = os.path.join(os.environ['DATA_DIR'], 'retracked')
        if not os.path.exists(args.dst):
            default_destination_ok = False
    else:
        default_destination_ok = False

    if not default_destination_ok:
        raise NotImplementedError('must pass --dst argument if DATA_DIR env' +
            'ironment variable is not set, or $DATA_DIR/retracked not a path')


src_id2containing_path = dict()
dst_id2containing_path = dict()

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
