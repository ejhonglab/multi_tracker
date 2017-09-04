#!/usr/bin/env python

from __future__ import division
import rospy
import time
import os
import subprocess
import numpy as np

class SaveBag:
    def __init__(self):
        rospy.init_node('save_delta_video', log_level=rospy.INFO)
        
        # TODO maybe default to saving all?
        self.topics = rospy.get_param('multi_tracker/delta_video/topics', [])
        if len(self.topics) == 0:
            # TODO maybe just fail? (reason for not is so this node can be 
            # left required in launch files)
            rospy.logwarn('NOT SAVING ANY TOPICS TO BAGFILE! You must ' + \
                'specify a list of topics as a parameter called ' + \
                'multi_tracker/delta_video/topics, if you wish to save ' + \
                'data, otherwise, there is no point in this node.')

        node_name = rospy.get_name()
        last_name_component = node_name.split('_')[-1]

        # but bewarned that this will also get the integer appended via anonymous flag
        # which will cause this to not save anything, because no topics will have
        # this random number appended to them upstream of this node...
        
        # TODO delete me
        rospy.logwarn('topics being saved in node ' + rospy.get_name() + ': ' + str(self.topics))
        
        self.record_length_seconds = 3600 * rospy.get_param('multi_tracker/record_length_hours', 24)
        
        # TODO break into utility function?
        self.experiment_basename = rospy.get_param('multi_tracker/experiment_basename', None)
        generated_basename = False
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this tracker run may differ!' + \
                ' Run the set_basename.py node along with others to fix this.')
            self.experiment_basename = time.strftime('%Y%m%d_%H%M%S', time.localtime())
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
        self.processRosbag = None
        rospy.on_shutdown(self.on_shutdown)
        
        # hacky. see TODOs in delta_video_simplebuffer.py
        self.time_start = 0
        # do w/o numpy?
        while np.isclose(self.time_start, 0.0):
            self.time_start = rospy.Time.now().to_sec()

    
    def on_shutdown(self):
        self.stop_recording()
        
    
    def start_recording(self):
        rospy.loginfo('Saving bag file: %s' % (self.bag_filename))
        cmdline = ['rosbag', 'record','-O', self.bag_filename]
        cmdline.extend(self.topics)
        print cmdline
        # TODO why did he take this approach w/ the preexec_fn?
        #self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
        self.processRosbag = subprocess.Popen(cmdline)
    
     
    def stop_recording(self):
        # TODO faced error where self.processRosbag was still None when this was called
        # that indicative of other problem?
        if not self.processRosbag is None:
            #subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
            self.processRosbag.kill()
            rospy.loginfo('Closed bag file.')
                
    
    def main(self):
        self.start_recording()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.time_start
            if t > self.record_length_seconds:
                # TODO maybe now i should check that process
                # is killed there?
                self.stop_recording()      
                break
        

if __name__ == '__main__':
    savebag = SaveBag()
    savebag.main()
    
