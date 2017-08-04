#!/usr/bin/env python

import rospy
import time
import os
import subprocess
import shutil
import glob

class SaveParams:
    def __init__(self):
        rospy.init_node('SaveParams', log_level=rospy.INFO)
        rospy.sleep(1)
        
        self.experiment_basename = rospy.get_param('multi_tracker/experiment_basename', None)
        generated_basename = False
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this tracker run may differ!' + \
                ' Run the set_basename.py node along with others to fix this.')
            self.experiment_basename = time.strftime("%Y%m%d_%H%M%S_N1", time.localtime())
            generated_basename = True
        
        suffix = '_parameters.yaml'
        filename = self.experiment_basename +'_'+ time.strftime('%Y%m%d_%H%M%S', time.localtime()) \
            + suffix
        
        if rospy.get_param('multi_tracker/explicit_directories', False):
            directory = os.path.expanduser( rospy.get_param('multi_tracker/data_directory') )
        else:
            directory = os.path.join(os.getcwd(), self.experiment_basename)
        
        # TODO make directory in a central place? set exp_basename?
        # or copy in each node? may need to explicitly fail then if not there as a 
        # result of central code
        if not os.path.exists(directory):
            os.makedirs(directory)
            if generated_basename:
                rospy.set_param('multi_tracker/experiment_basename', self.experiment_basename)
    
        self.filename = os.path.join(directory, filename)

        try:
            source_dir = rospy.get_param('source_directory')
            there = glob.glob(os.path.join(source_dir, '*' + suffix))
        except KeyError:
            there = []
            pass

        if len(there) > 0:
            here = glob.glob(os.path.join(directory, '*' + suffix))
            for f in there:
                if not f in here:
                    shutil.copy(os.path.join(source_dir, f), directory)
        
        # TODO explicitly check all nodes are loaded properly somehow?
        # that would be cleaner
        rospy.sleep(10.)

        # TODO not warn
        rospy.logwarn('Saving parameters to : %s' % (self.filename))
        cmdline = ['rosparam', 'dump', self.filename]
        self.processRosparam = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
        
        rospy.sleep(2.)
        subprocess.os.killpg(self.processRosparam.pid, subprocess.signal.SIGINT)
        rospy.logwarn('Closed process for saving ROS params.')
        
    
if __name__ == '__main__':
    saveparams = SaveParams()
    
