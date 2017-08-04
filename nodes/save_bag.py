#!/usr/bin/env python

from __future__ import division
import rospy
from optparse import OptionParser
import tf
import sys
import time, os, subprocess
import threading
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import imp

###############################################################################
#
class SaveBag:
    def __init__(self, config):
        # TODO configure this from ros yaml as well
        # TODO rename basename since we use basename so much elsewhere, to avoid confusion
        self.topics = config.topics
        self.record_length_seconds = 3600 * rospy.get_param('multi_tracker/record_length_hours', 24)
        
        # TODO break into utility function?
        self.experiment_basename = rospy.get_param('multi_tracker/experiment_basename', None)
        generated_basename = False
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this tracker run may differ!' + \
                ' Run the set_basename.py node along with others to fix this.')
            self.experiment_basename = time.strftime("%Y%m%d_%H%M%S_N1", time.localtime())
            generated_basename = True

        filename = self.experiment_basename + '_delta_video.bag'
        if rospy.get_param('multi_tracker/explicit_directories', False):
            directory = os.path.expanduser( rospy.get_param('multi_tracker/data_directory') )
        else:
            directory = os.path.join(os.getcwd(), self.experiment_basename)

        if not os.path.exists(directory):
            # TODO this could run into concurrency issues. lock somehow?
            os.makedirs(directory)
            if generated_basename:
                rospy.set_param('multi_tracker/experiment_basename', self.experiment_basename)

        self.bag_filename = os.path.join(directory, filename)
        # TODO does this need to go back to python time?
        self.time_start = rospy.Time.now()
        self.processRosbag = None
        rospy.on_shutdown(self.OnShutdown_callback)

    def OnShutdown_callback(self):
        self.StopRecordingBag()
        
    def StartRecordingBag(self):
        rospy.logwarn('Saving bag file: %s' % (self.bag_filename))
        cmdline = ['rosbag', 'record','-O', self.bag_filename]
        # TODO how to pass a list of something with ros params? type for that?
        cmdline.extend(self.topics)
        print cmdline
        self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
    
    def StopRecordingBag(self):
        subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
        rospy.logwarn('Closed bag file.')
                
    def Main(self):
        savebag.StartRecordingBag()
        rate = rospy.Rate(0.01)
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - self.time_start).to_sec()
            if t > self.record_length_seconds:
                self.StopRecordingBag()      
                return
        

if __name__ == '__main__':
    parser = OptionParser()
    # TODO centralize config and get rid of option
    parser.add_option("--config", type="str", dest="config", default='',
                        help="filename of configuration file")
    (options, args) = parser.parse_args()
    
    try:
        configuration = imp.load_source('configuration', options.config)
        print "Loaded configuration: ", options.config
    except: # look in home directory for config file
        if rospy.get_param('multi_tracker/explicit_directories', False):
            home_directory = os.path.expanduser( rospy.get_param('multi_tracker/home_directory') )
            config_file = os.path.join(home_directory, options.config)
            configuration = imp.load_source('configuration', config_file)
            print "Loaded configuration: ", config_file
        else:
            raise IOError(config_path + ' not found. Try launching tracking' + \
                ' from a directory with all required configuration files with ROS_HOME=`pwd`,' + \
                ' or set the multi_tracker/explicit_directories parameter in ' + \
                'tracker_parameters.yaml to true.')
    
    config = configuration.Config()

    # TODO why is this not init_node-d in __init__ of the class?
    # tracker does it that way; havent checked elsewhere yet.
    rospy.init_node('save_delta_video', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag(config)
    savebag.Main()
    
