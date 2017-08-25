#!/usr/bin/env python

import rospy
import time
import os
import subprocess
import shutil
import glob

class SaveParams:
    def __init__(self):
        rospy.init_node('save_params', log_level=rospy.INFO)
        rospy.sleep(1)
        
        self.experiment_basename = rospy.get_param('multi_tracker/experiment_basename', None)
        generated_basename = False
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this tracker run may differ!' + \
                ' Run the set_basename.py node along with others to fix this.')
        
            try:
                node_num = int(rospy.get_name().split('_')[-1])
            except ValueError:
                node_num =1
            
            # break out node_num getting function into utility module
            self.experiment_basename = time.strftime('%Y%m%d_%H%M%S_N' + str(node_num), \
                time.localtime())
            generated_basename = True

        self.namespace = rospy.get_param('~namespace', None)
        
        suffix = '_parameters.yaml'
        default_filename = self.experiment_basename +'_'+ time.strftime('%Y%m%d_%H%M%S', \
            time.localtime()) + suffix
        filename = rospy.get_param('~filename', default_filename)
        
        if rospy.get_param('multi_tracker/explicit_directories', False):
            directory = os.path.expanduser(rospy.get_param('multi_tracker/data_directory'))
        
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

        self.wait_s = rospy.get_param('~wait_s', 10.)
        
        # TODO explicitly check all nodes are loaded properly somehow?
        # that would be cleaner
        rospy.sleep(self.wait_s)

        # just stay alive and redump if they change? (rois)

        rospy.loginfo('Saving parameters to : %s' % (self.filename))
        cmdline = ['rosparam', 'dump', self.filename]

        if not self.namespace is None:
            our_ns = rospy.get_namespace()
            
            if our_ns[-1] == '/':
                ns = our_ns + self.namespace
            else:
                ns = our_ns + '/' + self.namespace
            
            #rospy.loginfo('saving parameters in namespace ' + ns)
            cmdline += [ns]

        self.processRosparam = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
        
        rospy.sleep(2.)
        subprocess.os.killpg(self.processRosparam.pid, subprocess.signal.SIGINT)
        rospy.loginfo('Closed process for saving ROS params.')
        
    
if __name__ == '__main__':
    saveparams = SaveParams()
    
