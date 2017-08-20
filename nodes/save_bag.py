#!/usr/bin/env python

from __future__ import division
import rospy
import time, os, subprocess

###############################################################################
#
class SaveBag:
    def __init__(self):
        # TODO maybe default to saving all?
        self.topics = rospy.get_param('multi_tracker/delta_video/topics', [])
        if len(self.topics) == 0:
            # TODO maybe just fail? (reason for not is so this node can be 
            # left required in launch files)
            rospy.logwarn('NOT SAVING ANY TOPICS TO BAGFILE! You must ' + \
                'specify a list of topics as a parameter called ' + \
                'multi_tracker/delta_video/topics, if you wish to save ' + \
                'data, otherwise, there is no point in this node.')
        
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
        rospy.loginfo('Saving bag file: %s' % (self.bag_filename))
        cmdline = ['rosbag', 'record','-O', self.bag_filename]
        # TODO how to pass a list of something with ros params? type for that?
        cmdline.extend(self.topics)
        print cmdline
        self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
    
    def StopRecordingBag(self):
        # TODO faced error where self.processRosbag was still None when this was called
        # that indicative of other problem?
        if not self.processRosbag is None:
            subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
            rospy.loginfo('Closed bag file.')
                
    def Main(self):
        self.StartRecordingBag()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - self.time_start).to_sec()
            if t > self.record_length_seconds:
                self.StopRecordingBag()      
                return
        

if __name__ == '__main__':
    # TODO why is this not init_node-d in __init__ of the class?
    # tracker does it that way; havent checked elsewhere yet.
    rospy.init_node('save_delta_video', log_level=rospy.INFO)
    savebag = SaveBag()
    savebag.Main()
    
