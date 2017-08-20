#!/usr/bin/env python

import sys
import os
import rospy

# what does this log_level do?
rospy.init_node('publish_original_basename', log_level=rospy.INFO)

# remove extra args roslaunch adds
argv = [a for a in sys.argv[1:] if '__' != a[:2] and not ':=' in a]

if len(argv) < 1:
    filename_or_fullpath = os.getcwd()

elif len(argv) > 1:
    raise ValueError('Usage: pass one or zero path / bagfile name arguments. If path, the directory must only have one bagfile in it.')

else:
    filename_or_fullpath = os.path.abspath(os.path.expanduser(argv[0]))

bagfile_suffix = '_delta_video.bag'
if not bagfile_suffix in filename_or_fullpath:
    import glob
    possible_files = glob.glob(os.path.join(filename_or_fullpath, '*' + bagfile_suffix))
    
    if len(possible_files) < 1:
        raise ValueError(filename_or_fullpath + ' was not a properly named bagfile, nor was one in that directory.')

    elif len(possible_files) > 1:
        raise IOError('too many bagfiles in input directory. can not disambiguate.')

    filename_or_fullpath = possible_files[0]

filename = os.path.split(filename_or_fullpath)[-1]
original_basename = filename[:-1 * len(bagfile_suffix)]
# further down?
rospy.set_param('original_basename', original_basename)
# TODO test
rospy.set_param('source_directory', os.path.join(os.path.split(filename_or_fullpath)[:-1]))

