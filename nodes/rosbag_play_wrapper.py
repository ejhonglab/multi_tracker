#!/usr/bin/env python

import rospy
import argparse
import os
import sys
import glob
import subprocess

"""
PLays any bag file in the current directory, directory passed, and also existing behavior of 
passed filename (absolute or relative).
"""
# TODO test relative paths work for directories and files

class RosbagPlayWrapper:
    def __init__(self, args):
        if args.path[-4:] == '.bag':
            self.bag_filename = args.path
        
        else:
            bags_in_path = glob.glob(os.path.join(args.path, '*.bag'))

            if len(bags_in_path) == 0:
                raise IOError('no bagfiles found in path')

            elif len(bags_in_path) > 1:
                raise IOError('more than one bagfile in path. pass path to file (rather than directory) or put it in its own directory, to resolve ambiguity.')
            
            self.bag_filename = os.path.abspath(os.path.expanduser(bags_in_path[0]))

        self.topic_in = rospy.get_param('~topic_in', 'multi_tracker/delta_video')
        self.topic_out = rospy.get_param('~topic_out', 'multi_tracker/delta_video')

        # reformat remaining args into a list to be passed to Popen
        dict_args = vars(args)
        args_dict = dict((('--' + k, v) for k, v in dict_args.items() if k != 'path' and not v is None))
        pairs_list = [[k, v] for k, v in args_dict.items() if not isinstance(v, bool)]
        flags_list = [k for k, v in args_dict.items() if isinstance(v, bool) and v]
        # TODO this doesn't seem to be working... couldn't seem to remap w/ it
        self.passthrough_args = flags_list + [item for sublist in pairs_list for item in sublist]
        
        self.rosbag_process = None
        rospy.on_shutdown(self.on_shutdown)
        self.play()

    def on_shutdown(self):
        self.stop()
        
    def play(self):
        rospy.loginfo('starting process to play bag file: %s' % (self.bag_filename))
        # remapping will only work assuming that topic is in the bag
        # TODO appropriate check / error message if topic not in bag
        # need to remap from global?
        cmdline = ['rosbag', 'play', self.bag_filename, self.topic_in + ':=' + self.topic_out]
        cmdline.extend(self.passthrough_args)
        self.rosbag_process = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
        # TODO just spin until Popen finishes? maybe make a blocking call?
        rospy.spin()
    
    def stop(self):
        if not self.rosbag_process is None:
            subprocess.os.killpg(self.rosbag_process.pid, subprocess.signal.SIGINT)
            rospy.loginfo('rosbag play process killed from wrapper.')


if __name__ == '__main__':
    # TODO dont require a node? does rosbag play need master?
    # TODO this node might not be getting killed correctly w/ ctrl-c? handler?
    rospy.init_node('rosbag_play_wrapper', log_level=rospy.INFO)

    # any easier way to get the optional positional dir / file argument
    # in a variety of places, while passing through all rosbag arguments?
    parser = argparse.ArgumentParser()
    
    # make this nargs '*' if you want to adapt this to play more than one bagfile
    # might require some other minor changes
    parser.add_argument('path', nargs='?', default=os.getcwd())
    
    # all the rosbag defaults we want to pass through
    # only supporting a subset for now
    parser.add_argument('--clock', action='store_true')
    parser.add_argument('--pause', action='store_true')
    parser.add_argument('--quiet', action='store_true')
    
    # i assume these both just keep it as a string?
    # work with both --rate=S and --rate S formats?
    parser.add_argument('--rate', action='store')
    parser.add_argument('--start', action='store')
    
    # TODO is it possible the rosbag play queue is the main place losing messages?
    # because there is the --queue arg to rosbag play
    # wish it was possible / easier to diagnose...
    # need to remove some the of the extra parameters that ROS(launch?) adds
    argv = [a for a in sys.argv[1:] if '__' != a[:2] and not ':=' in a]
    args = parser.parse_args(argv)
    
    wrapper = RosbagPlayWrapper(args)
    
