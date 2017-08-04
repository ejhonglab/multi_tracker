#!/usr/bin/env python
from __future__ import division
import rospy
import rosparam
import copy
import cv2
import numpy as np
import threading
# TODO
import dynamic_reconfigure.server
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header, String

from multi_tracker.msg import Contourinfo, Contourlist, DeltaVid
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService

import time
import os
import image_processing

import matplotlib.pyplot as plt

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: /camera/image_raw

# for firefley cameras, camera1394 does not provide timestamps but otherwise works
# use point grey drivers
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: /camera/image_mono

# The main tracking class, a ROS node
class Compressor:
    def __init__(self):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
        # TODO set via default yaml?
        # share this default setting code?
        # TODO does camera need to be global if multiple multi_trackers are going to operate on it?
        # idiomatic way to do that? is what floris did the best?
        self.params = { 'image_topic'       : 'camera/image_raw',
                        'threshold'         : 10,
                        'camera_encoding'   : 'mono8', # fireflies are bgr8, basler gige cams are mono8
                        'max_change_in_frame'       : 0.2,
                        'roi_l'                     : 0,
                        'roi_r'                     : -1,
                        'roi_b'                     : 0,
                        'roi_t'                     : -1,
                        'circular_mask_x'           : 'none',
                        'circular_mask_y'           : 'none',
                        'circular_mask_r'           : 'none',
                        }
        for parameter, value in self.params.items():
            try:
                p = 'multi_tracker/delta_video/' + parameter
                self.params[parameter] = rospy.get_param(p)
            except:
                print 'Using default parameter: ', parameter, ' = ', value
        
        # initialize the node
        rospy.init_node('delta_compressor')
        # TODO the self.nodename variable that used to be here wasnt something used by rospy
        # under the hood is it?
        # TODO why does rospy.Time.now() jump from 0 to ~149998...???
        # this solution seems hacky and i wish i didn't have to do it...
        self.time_start = 0
        while np.isclose(self.time_start, 0.0):
            self.time_start = rospy.Time.now().to_sec()
        
        self.save_data = rospy.get_param('multi_tracker/delta_video/save_data', True)
        self.record_length_seconds = 3600 * rospy.get_param('multi_tracker/record_length_hours', 24)

        self.use_original_timestamp = rospy.get_param('multi_tracker/retracking_original_timestamp', False)
        self.experiment_basename = rospy.get_param('multi_tracker/experiment_basename', None)
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this tracker run may differ!' + \
                ' Run the set_basename.py node along with others to fix this.')
            self.experiment_basename = time.strftime("%Y%m%d_%H%M%S_N1", time.localtime())
        
        # Publishers - publish pixel changes
        self.pubDeltaVid = rospy.Publisher('multi_tracker/delta_video', DeltaVid, queue_size=30)

        # determine what kind of differences from background frame we are interested in
        tracking_fn = rospy.get_param('multi_tracker/tracker/image_processor')
        if tracking_fn == 'dark_objects_only':
            self.sign = -1
        elif tracking_fn == 'light_objects_only':
            self.sign = 1
        else:
            # will just use absdiff if the sign of the deviations of interest isn't obvious
            # from the name of the tracking function
            self.sign = 0
        
        # background reset service
        self.reset_background_flag = False
        self.reset_background_service = rospy.Service('multi_tracker/reset_background', resetBackgroundService, self.reset_background)
        
        self.cvbridge = CvBridge()
        self.imgScaled      = None
        self.backgroundImage = None
        self.background_img_filename = 'none'
        
        # buffer locking
        self.lockBuffer = threading.Lock()
        self.image_buffer = []
        self.framestamp = None
        
        self.current_background_img = 0
        
        # Subscriptions - subscribe to images, and tracked objects
        self.image_mask = None 
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subImage = rospy.Subscriber(self.params['image_topic'], Image, self.image_callback, queue_size=5, buff_size=2*sizeImage, tcp_nodelay=True)

    def reset_background(self, service_call):
        self.reset_background_flag = True
        return 1
        
    def image_callback(self, rosimg):
        with self.lockBuffer:
            self.image_buffer.append(rosimg)

    def process_image_buffer(self, rosimg):
        if self.framestamp is not None:
            self.dtCamera = (rosimg.header.stamp - self.framestamp).to_sec()
        else:
            self.dtCamera = 0.03
        self.framenumber = rosimg.header.seq
        self.framestamp = rosimg.header.stamp
        
        # Convert the image.
        try:
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
            img = np.zeros((320,240))

        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'], self.params['roi_l']:self.params['roi_r']]
        self.shapeImage = self.imgScaled.shape # (height,width)
        
        if self.params['circular_mask_x'] != 'none':
            if self.image_mask is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                cv2.circle(self.image_mask,(self.params['circular_mask_x'], self.params['circular_mask_y']), int(self.params['circular_mask_r']), [1,1,1], -1)
            self.imgScaled = self.image_mask*self.imgScaled

        def background_png_name():
            # TODO fix nodenum. derive from enclosing namespace if possible.
            nodenum = 1
            if self.use_original_timestamp:
                background_img_filename = self.experiment_basename + time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M%.png', time.localtime(rospy.Time.now().to_sec()))
            else:
                background_img_filename = self.experiment_basename + time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M%.png', time.localtime())

            data_directory = os.path.expanduser( rospy.get_param('multi_tracker/data_directory') )
            return os.path.join(data_directory, background_img_filename)
        
########### image processing function ##############################################################
        
        # If there is no background image, grab one, and move on to the next frame
        if self.backgroundImage is None:
            self.backgroundImage = copy.copy(self.imgScaled)
            self.background_img_filename = background_png_name()
            if self.save_data:
                cv2.imwrite(self.background_img_filename, self.backgroundImage)
            self.current_background_img += 1
            return

        # TODO will this be true by default? if so, is it always saving two images?
        # maybe get rid of the above?
        if self.reset_background_flag:
            self.backgroundImage = copy.copy(self.imgScaled)
            self.background_img_filename = background_png_name()
            if self.save_data:
                cv2.imwrite(self.background_img_filename, self.backgroundImage)
            self.current_background_img += 1
            self.reset_background_flag = False
            return

        # calculate the difference from the background
        # sign set in __init__ based on name of tracking function (defaults to 0)
        # TODO test
        if self.sign == -1:
            self.diff = self.backgroundImage - self.imgScaled
        elif self.sign == 1:
            self.diff = self.imgScaled - self.backgroundImage
        else:
            self.diff = cv2.absdiff(self.imgScaled, self.backgroundImage)

        changed_pixels = np.where(self.diff > self.params['threshold'])
        delta_msg = DeltaVid()
        header  = Header(stamp=self.framestamp,frame_id=str(self.framenumber))
        delta_msg.header = header
        delta_msg.background_image = self.background_img_filename
        if len(changed_pixels[0]) > 0:
            delta_msg.xpixels = changed_pixels[0].tolist()
            delta_msg.ypixels = changed_pixels[1].tolist()
            delta_msg.values = self.imgScaled[changed_pixels].reshape(len(changed_pixels[0])).tolist()
        else:
            delta_msg.xpixels = [0]
            delta_msg.ypixels = [0]
            #delta_msg.values = [0]
        self.pubDeltaVid.publish(delta_msg)
        
        # if the thresholded absolute difference is too large, reset the background
        if len(changed_pixels[0]) / (self.diff.shape[0] * self.diff.shape[1]) > self.params['max_change_in_frame']:
            self.reset_background_flag = True

        #self.backgroundImage[delta_msg.xpixels, delta_msg.ypixels] = delta_msg.values 
    
    def Main(self):
        while (not rospy.is_shutdown()):
            t = rospy.Time.now().to_sec() - self.time_start
            if t > self.record_length_seconds:
                cv2.destroyAllWindows()
                return
            
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.image_buffer) > 0:
                    self.process_image_buffer(self.image_buffer.pop(0))
                
                pt = (rospy.Time.now() - time_now).to_sec()
                if len(self.image_buffer) > 3:
                    rospy.logwarn("Delta video processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.image_buffer))
            
        # TODO why here?
        cv2.destroyAllWindows()

#####################################################################################################
    
if __name__ == '__main__':
    compressor = Compressor()
    compressor.Main()
