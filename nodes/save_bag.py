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
        basename = config.basename
        directory = config.directory
        self.topics = config.topics
        self.record_length_seconds = 3600 * rospy.get_param('multi_tracker/record_length_hours', 24)
        self.use_original_timestamp = rospy.get_param('multi_tracker/retracking_original_timestamp', False)
        
        # TODO does this need to go back to python time?
        self.time_start = rospy.Time.now()
        
        experiment_basename = rospy.get_param('multi_tracker/experiment_basename', 'none')
        if experiment_basename == 'none':
            # TODO fix
            nodenum = 1
            if self.use_original_timestamp:
                self.experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime(rospy.Time.now()))
            else:
                self.experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime())
        
        filename = experiment_basename + '_' + basename + '.bag'

        # Make sure path exists.
        try:
            os.makedirs(directory)
        except OSError:
            pass

        self.filenameBag = os.path.expanduser(os.path.join(directory, filename))
        self.processRosbag = None
        
        rospy.on_shutdown(self.OnShutdown_callback)

    def OnShutdown_callback(self):
        self.StopRecordingBag()
        
    def StartRecordingBag(self):
        rospy.logwarn('Saving bag file: %s' % (self.filenameBag))
        cmdline = ['rosbag', 'record','-O', self.filenameBag]
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
            t = rospy.Time.now() - self.time_start
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
        home_directory = os.path.expanduser( rospy.get_param('multi_tracker/home_directory') )
        config_file = os.path.join(home_directory, options.config)
        configuration = imp.load_source('configuration', config_file)
        print "Loaded configuration: ", config_file
    
    config = configuration.Config()

    # TODO why is this not init_node-d in __init__ of the class?
    # tracker does it that way; havent checked elsewhere yet.
    rospy.init_node('SaveBag', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag(config)
    savebag.Main()
    
