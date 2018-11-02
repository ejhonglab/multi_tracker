#!/usr/bin/env python

import os
import sys
import subprocess

import rospy


def get_single_filename(path, suffix, filetype):
    """Checks that there is only one of the given filetype in the specified
    path, and returns a path to that single file.

    If suffix is in path, just returns the path.
    """
    if not suffix in path:
        import glob
        possible_files = \
            glob.glob(os.path.join(path, '*' + suffix))
        
        if len(possible_files) < 1:
            raise ValueError(path + ' was not a properly ' +
                'named {}, nor was one in that directory.'.format(filetype))

        elif len(possible_files) > 1:
            raise IOError('too many {}s in input directory.'.format(filetype) +
                ' can not disambiguate.')

        path = possible_files[0]

    return path


if __name__ == '__main__':
    # what does this log_level do?
    rospy.init_node('publish_original_basename', log_level=rospy.INFO)

    # remove extra args roslaunch adds
    argv = [a for a in sys.argv[1:] if '__' != a[:2] and not ':=' in a]

    if len(argv) < 1:
        filename_or_fullpath = os.getcwd()

    elif len(argv) > 1:
        raise ValueError('Usage: pass one or zero path / bagfile name ' +
            'arguments. If path, the directory must only have one ' +
            'bagfile in it.')

    else:
        filename_or_fullpath = os.path.abspath(os.path.expanduser(argv[0]))


    bagfile_suffix = '_delta_video.bag'
    bagfile = get_single_filename(filename_or_fullpath,
                                               bagfile_suffix,
                                               'bagfile')

    # Handling this in Python because:
    # 1) Finding parameter file name is not possible within launch file
    #    semantics.
    # 2) This should set parameters before tracking launch file is
    #    separately invoked, anyway.
    paramyaml_suffix = '_parameters.yaml'
    paramyaml = get_single_filename(filename_or_fullpath,
                                               paramyaml_suffix,
                                               'parameter YAML')
    rospy.loginfo('Loading parameters from: {}'.format(paramyaml))
    # TODO prevent this from loading ROS params if it's a problem:
    # (/run_id, /rosversion, /rosdistro, /roslaunch)
    subprocess.Popen(['rosparam', 'load', paramyaml])

    filename = os.path.split(bagfile)[-1]
    original_basename = filename[:-1 * len(bagfile_suffix)]
    # further down?
    rospy.set_param('original_basename', original_basename)

    # split only splits between deepest directory and the file to the right of
    # the separator. [0] is everything on the left of that separator
    retracking_input_directory = os.path.split(bagfile)[0]
    rospy.set_param('source_directory', retracking_input_directory)

