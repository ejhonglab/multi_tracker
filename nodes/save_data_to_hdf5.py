#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import os
import time
import threading

import numpy as np
import h5py
import rospy
import atexit
import errno

from multi_tracker.msg import Trackedobject, Trackedobjectlist


# TODO maybe save roi information here to not require snapshot_params to delay
# until rois are set?  or just for redundancy?

class DataListener:
    def __init__(self, info='data information'):
        rospy.init_node('save_hdf5_data', log_level=rospy.INFO)
        rospy.sleep(0.5)

        node_name = rospy.get_name()
        last_name_component = node_name.split('_')[-1]
        # TODO see discussion in this portion in save_bag.py
        try:
            self.pipeline_num = int(last_name_component)
            remap_topics = True
        
        except ValueError:
            self.pipeline_num = 1
            remap_topics = False

        tracked_object_topic = 'multi_tracker/tracked_objects'
        if remap_topics:
            tracked_object_topic = '{}_{}'.format(tracked_object_topic,
                                                  self.pipeline_num)

        self.subTrackedObjects = rospy.Subscriber(tracked_object_topic,
                                                  Trackedobjectlist,
                                                  self.tracked_object_callback,
                                                  queue_size=300)
        
        # TODO maybe append _<n> to this?
        self.experiment_basename = \
            rospy.get_param('multi_tracker/experiment_basename', None)

        generated_basename = False
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this ' + 
                'tracker run may differ!')
            self.experiment_basename = time.strftime('%Y%m%d_%H%M%S',
                time.localtime())
            generated_basename = True
        
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

        filename = os.path.join(directory, self.experiment_basename + '_N' + 
            str(self.pipeline_num) + '_trackedobjects.hdf5')

        rospy.loginfo('Saving hdf5 data to: ' + filename)
        

        # TODO should the default here be to record indefinitely? (i.e. -1)
        hrs_to_record = rospy.get_param('multi_tracker/record_length_hours', 24)
        # TODO document this behavior + implement in other nodes using this
        # param
        if hrs_to_record > 0:
            self.record_length_seconds = 3600 * hrs_to_record
        else:
            self.record_length_seconds = -1

        self.time_start = rospy.Time.now()
        
        self.buffer = []
        self.array_buffer = []
        # set up thread locks
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        
        self.chunk_size = 5000
        self.hdf5 = h5py.File(filename, 'w')
        # helps prevent file corruption if closed improperly
        self.hdf5.swmr_mode = True 
        self.hdf5.attrs.create("info", info)
        
        self.data_to_save = [
            'objid',
            'header.stamp.secs',
            'header.stamp.nsecs', 
            'header.frame_id', 
            'position.x', 
            'position.y', 
            'position.z', 
            'velocity.x',
            'velocity.y',
            'velocity.z',
            'angle',
            'size',
            'covariance',
            'measurement.x',
            'measurement.y'
        ]

        self.data_format = {
            'objid': int,
            'header.stamp.secs': int,
            'header.stamp.nsecs': int, 
            'header.frame_id': int, 
            'position.x': float, 
            'position.y': float, 
            'position.z': float, 
            'velocity.x': float,
            'velocity.y': float,
            'velocity.z': float,
            'angle': float,
            'size': float,
            'covariance': float,
            'measurement.x': float,
            'measurement.y': float
        }
                            
        self.dtype = [(data,self.data_format[data])
                      for data in self.data_to_save]
        
        self.hdf5.create_dataset('data',
                                 (self.chunk_size, 1),
                                 maxshape=(None,1),
                                 dtype=self.dtype)

        self.hdf5['data'].attrs.create('current_frame', 0)
        self.hdf5['data'].attrs.create('line', 0)
        self.hdf5['data'].attrs.create('length', self.chunk_size)
        
    def add_chunk(self):
        length = self.hdf5['data'].attrs.get('length')
        new_length = length + self.chunk_size
        self.hdf5['data'].resize(new_length, axis=0)
        self.hdf5['data'].attrs.modify('length', new_length)
            
    def save_array_data(self):
        newline = self.hdf5['data'].attrs.get('line') + 1
        nrows_to_add = len(self.array_buffer)
        
        self.hdf5['data'].attrs.modify('line', newline+nrows_to_add)
        if newline+nrows_to_add >= self.hdf5['data'].attrs.get('length')-50:
            self.hdf5.flush()
            self.add_chunk()
        
        self.hdf5['data'][newline:newline+nrows_to_add] = self.array_buffer
        # TODO (?)
        self.array_buffer = []
                                                   
    def tracked_object_callback(self, tracked_objects):
        with self.lockBuffer:
            for tracked_object in tracked_objects.tracked_objects:
                # TODO delete this comment. where are stamps in header defined?
                # using correct time source?
                a = np.array([(tracked_object.objid,
                               tracked_object.header.stamp.secs,
                               tracked_object.header.stamp.nsecs,
                               tracked_object.header.frame_id,

                               tracked_object.position.x,
                               tracked_object.position.y,
                               tracked_object.position.z,
                               
                               tracked_object.velocity.x,
                               tracked_object.velocity.y,
                               tracked_object.velocity.z,

                               tracked_object.angle,
                               tracked_object.size,
                               tracked_object.covariance,

                               tracked_object.measurement.x,
                               tracked_object.measurement.y
                              )], dtype=self.dtype)
                self.array_buffer.append(a)
        
    def process_buffer(self):
        self.save_array_data()
            
    def main(self):
        atexit.register(self.stop_saving_data)
        while not rospy.is_shutdown():
            # TODO delete this comment. is there a reason this was previously
            # not using ros time?
            t = (rospy.Time.now() - self.time_start).to_sec()

            if (self.record_length_seconds > 0 and
                t > self.record_length_seconds):

                # TODO should still work without this, right? maybe just remove
                # and test it still works? oh... maybe that HDF5 error was
                # caused by double closing, since the atexit call is also
                # invoked?)
                #self.stop_saving_data()
                return
            
            # TODO why locking here and not further down? (less probably better)
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.array_buffer) > 0:
                    self.process_buffer()

                pt = (rospy.Time.now() - time_now).to_sec()
                if len(self.buffer) > 9:
                    rospy.logwarn('Data saving processing time exceeds ' +
                        'acquisition rate. Processing time: %f, Buffer: %d',
                        pt, len(self.buffer))
    
    # TODO fix hdf5 bug here (maybe just requires updating HDF5 / h5py?)
    # see note above
    def stop_saving_data(self):
        self.hdf5.close()
        # presumably printing because the logging facilities will be inoperable
        # by the time this is called
        print('save_data_to_hdf5 shut down nicely')
        
if __name__ == '__main__':
    datalistener = DataListener()
    datalistener.main()
