#!/usr/bin/env python

from __future__ import division

import time
import os
import subprocess
import errno

import numpy as np
import rospy
import rostopic


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
        # but bewarned that this will also get the integer appended via
        # anonymous flag which will cause this to not save anything, because no
        # topics will have this random number appended to them upstream of this
        # node...
        last_name_component = node_name.split('_')[-1]
        

        # TODO should the default here be to record indefinitely? (i.e. -1)
        # TODO document this behavior
        hrs_to_record = rospy.get_param('multi_tracker/record_length_hours', 24)

        # This logic is duplicated here and in save_data_to_hdf5 because some
        # launch files only use one or the other, at each of those counts on
        # whichever of these nodes that is running to stop the acquisition.
        if hrs_to_record > 0:
            self.record_length_seconds = 3600 * hrs_to_record
        else:
            self.record_length_seconds = -1
        
        # TODO break into utility function?
        self.experiment_basename = \
            rospy.get_param('multi_tracker/experiment_basename', None)

        self.compression = rospy.get_param('~compression', True)

        generated_basename = False
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this ' +
                'tracker run may differ!')

            self.experiment_basename = \
                time.strftime('%Y%m%d_%H%M%S', time.localtime())

            generated_basename = True

        filename = self.experiment_basename + '_delta_video.bag'
        if rospy.get_param('multi_tracker/explicit_directories', False):
            directory = os.path.expanduser(
                rospy.get_param('multi_tracker/data_directory'))

        else:
            directory = os.path.join(os.getcwd(), self.experiment_basename)

        try:
            os.makedirs(directory)
            if generated_basename:
                rospy.set_param('multi_tracker/experiment_basename', \
                    self.experiment_basename)

        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

        self.bag_filename = os.path.join(directory, filename)
        self.processRosbag = None
        rospy.on_shutdown(self.on_shutdown)

    
    def on_shutdown(self):
        self.stop_recording()
        
    
    def start_recording(self):
        rospy.loginfo('Saving bag file: %s' % (self.bag_filename))
        cmdline = ['rosbag', 'record','-O', self.bag_filename]

        if self.compression:
            # could try --lz4, if bz2 proves too resource intensive
            cmdline.append('--bz2')

        cmdline.extend(self.topics)
        self.processRosbag = \
            subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
    
     
    def stop_recording(self):
        if not self.processRosbag is None:
            subprocess.os.killpg(self.processRosbag.pid,
                                 subprocess.signal.SIGINT)
            rospy.loginfo('Closed bag file.')
            # maybe hold on to it in case we need to escalate shutdown signals?
            self.processRosbag = None
   

    def main(self):
        self.start_recording()

        for topic in self.topics:
            rospy.logwarn(rospy.get_name() + ' waiting on ' + topic + \
                ' to start record duration')
            recheck = False
            while not rospy.is_shutdown():
                if recheck:
                    time.sleep(0.5)

                # TODO failure mode if topics that dont currently exist are in
                # list to record? i want to support that w/o failure
                
                name = rospy.resolve_name(topic)
                msg_type, _, _ = rostopic.get_topic_class(name)
                if msg_type is None:
                    recheck = True
                    continue

                else:
                    rospy.wait_for_message(name, msg_type)
                    break
        
        if not rospy.is_shutdown():
            rospy.loginfo(rospy.get_name() + ' beginning record duration.')
            # hacky. see TODOs in delta_video_simplebuffer.py
            self.time_start = 0
            # do w/o numpy?
            # TODO does this need to go back to python time?
            while np.isclose(self.time_start, 0.0):
                self.time_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.time_start

            if (self.record_length_seconds > 0 and
                t > self.record_length_seconds):

                # TODO maybe now i should check that process
                # is killed there?
                # TODO atexit hook also a good idea here?
                self.stop_recording()      
                return
        

if __name__ == '__main__':
    savebag = SaveBag()
    savebag.main()
