#!/usr/bin/env python

import sys
import os
import rospy

# what does this log_level do?
rospy.init_node('publish_original_basename', log_level=rospy.INFO)

filename_or_fullpath = sys.argv[1]
filename = os.path.split(filename_or_fullpath)[-1]
bagfile_suffix = '_delta_video.bag'
if not bagfile_suffix in filename:
    raise ValueError(filename_or_fullpath + ' was not a properly named bagfile.')

original_basename = filename[:-1 * len(bagfile_suffix)]
# further down?
rospy.set_param('original_basename', original_basename)
