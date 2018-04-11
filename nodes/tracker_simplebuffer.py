#!/usr/bin/env python

from __future__ import division

import imp
import os
import threading
import time

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import dynamic_reconfigure.server
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header, String

from multi_tracker.msg import Contourinfo, Contourlist
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService
from multi_tracker.srv import addImageToBackgroundService

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: camera/image_raw

# For firefley cameras, camera1394 does not provide timestamps but otherwise
# works. Use point grey drivers.
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: camera/image_mono

# The main tracking class, a ROS node
class Tracker:
    def __init__(self):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        rospy.init_node('multi_tracker')
        rospy.sleep(1)
        
        # TODO load from a defaults.yaml?
        # default parameters (parameter server overides them)
        self.params = {
            'image_topic'           : 'camera/image_mono',
            'threshold'             : 20,
            'backgroundupdate'      : 0.001,
            'medianbgupdateinterval': 30,
            # fireflies are bgr8, basler gige cams are mono8
            'camera_encoding'       : 'mono8',
            'erode'                 : 1,
            'dilate'                : 2,
            'morph_open_kernel_size': 3,
            'max_change_in_frame'   : 0.2,
            'min_size'              : 5,
            'max_size'              : 200,
            'max_expected_area'     : 500,
            'denoise'               : True,
            # TODO what does this do? remove?
            'liveview'              : False,
            # see notes in delta_video_simplebuffer
            'roi_l'                 : 0, 
            'roi_r'                 : -1,
            'roi_b'                 : 0,
            'roi_t'                 : -1,
            '~circular_mask_x'      : None,
            '~circular_mask_y'      : None,
            '~circular_mask_r'      : None,
            # TODO implement
            '~roi_points'           : None,
            # use moments for x,y,area instead of fitted ellipse
            'use_moments'           : True,
            'record_length_hours'   : 24
        }

        # TODO break this code out into a utility function
        for parameter, default_value in self.params.items():
            use_default = False
            try:
                if parameter[0] == '~':
                    value = rospy.get_param(parameter)
                else:
                    p = 'multi_tracker/tracker/' + parameter
                    value = rospy.get_param(p)

                # for maintaining backwards compatibility w/ Floris' config
                # files that would use 'none' to signal default should be used.
                # may break some future use cases.
                if self.params[parameter] is None:
                    if isinstance(value, str):
                        use_default = True

            except KeyError:
                use_default = True
                
            if use_default:
                rospy.loginfo(rospy.get_name() + ' using default parameter: ' +
                    parameter + ' = ' + str(default_value))
                value = default_value
            
            if parameter[0] == '~':
                del self.params[parameter]
                parameter = parameter[1:]
            
            self.params[parameter] = value

        if self.params['min_size'] < 5:
            rospy.logfatal('only contours that can be fit with ellipses are ' + 
                'supported now. contours must have at least 5 pixels to be ' + 
                'fit. please increase min_size parameter.')

        # TODO maybe just reduce to debug flag and not save data in that case?
        self.save_data = rospy.get_param('multi_tracker/tracker/save_data',True)
        self.debug = rospy.get_param('multi_tracker/tracker/debug', False)
        self.record_length_seconds = (3600 * 
            rospy.get_param('multi_tracker/record_length_hours', 24))
        
        node_name = rospy.get_name()
        last_name_component = node_name.split('_')[-1]
        # TODO see discussion in this portion in save_bag.py
        try:
            self.pipeline_num = int(last_name_component)
            remap_topics = True
        
        except ValueError:
            # warn if?
            self.pipeline_num = 1
            remap_topics = False

        # TODO should the experiment_basename have the pipeline # in it? ->
        # should each fly's data be saved to a separate directory (probably
        # not?)?
        self.experiment_basename = \
            rospy.get_param('multi_tracker/experiment_basename', None)

        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this ' +
                'tracker run may differ!')
            self.experiment_basename = time.strftime('%Y%m%d_%H%M%S',
                time.localtime())

        # used by image_processing code that is spliced in with imp
        self.explicit_directories = \
            rospy.get_param('multi_tracker/explicit_directories', False)
        
        # initialize the node
        self.time_start = rospy.Time.now().to_sec()
        
        # background reset service
        self.reset_background_flag = False
        self.add_image_to_background_flag = False

        # TODO is this taking the right reset_background?
        # another function of similar name is loaded in here
        # with imp...
        self.reset_background_service = rospy.Service(
            'multi_tracker/tracker/reset_background',
            resetBackgroundService,
            self.reset_background
        )

        self.add_image_to_background_service = rospy.Service(
            'multi_tracker/tracker/add_image_to_background',
            addImageToBackgroundService,
            self.add_image_to_background
        )

        self.cvbridge = CvBridge()
        self.imgScaled      = None
        self.backgroundImage = None
        
        self.lockBuffer = threading.Lock()
        self.image_buffer = []
        self.framestamp = None
        
        tracked_object_topic = 'multi_tracker/tracked_objects'
        
        if remap_topics:
            suffix = '_' + str(self.pipeline_num)
            tracked_object_topic = tracked_object_topic + suffix

        else:
            suffix = ''

        self.pubContours = rospy.Publisher('multi_tracker/contours_{}'.format(
            self.pipeline_num), Contourlist, queue_size=300)
        
        self.image_mask = None 
        # TODO define dynamically
        # Size of header + data.
        sizeImage = 128+1024*1024*3 
        
        # TODO have delta_video also publish a cropped version for this
        # pipeline?
        self.subImage = rospy.Subscriber(self.params['image_topic'],
                Image, self.image_callback, queue_size=60, 
                buff_size=2*sizeImage, tcp_nodelay=True)
        
        # TODO launch from within a python script so i can actually
        # conditionally open their viewers (based on debug settings)?
        if self.debug:
            self.pub_mask = rospy.Publisher('multi_tracker/0_mask' + suffix,
                                            Image,
                                            queue_size=5)

            self.pub_threshed = rospy.Publisher(
                'multi_tracker/1_thresholded' + suffix, Image, queue_size=5)

            self.pub_denoised = rospy.Publisher(
                'multi_tracker/2_denoised' + suffix, Image, queue_size=5)

            self.pub_dilated = rospy.Publisher(
                'multi_tracker/3_dilated' + suffix, Image, queue_size=5)

            self.pub_eroded = rospy.Publisher('multi_tracker/4_eroded' + suffix,
                                              Image,
                                              queue_size=5)

            self.pub_processed = rospy.Publisher(
                'multi_tracker/processed_image' + suffix, Image, queue_size=5)

    
    def image_callback(self, rosimg):
        with self.lockBuffer:
            self.image_buffer.append(rosimg)

    # TODO get rid of argument if not using them? or necessary
    # for rospy service callback? (return too?)
    def reset_background(self, service_call):
        self.reset_background_flag = True
        return 1
    
    # same thing about arg?
    def add_image_to_background(self, service_call):
        self.add_image_to_background_flag = True
        return 1
        
    def process_image_buffer(self, rosimg):
        # TODO check for problems this dtCamera stuff might cause
        if self.framestamp is not None:
            self.dtCamera = (rosimg.header.stamp - self.framestamp).to_sec()
        else:
            self.dtCamera = 0.03
        self.framenumber = rosimg.header.seq
        self.framestamp = rosimg.header.stamp
        
        # Convert the image.
        try:
            # might need to change to bgr for color cameras
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough')

        except CvBridgeError, e:
            rospy.logwarn('Exception converting background image from ROS to' +
                ' OpenCV:  %s' % e)
            # TODO define dynamically. is this even consistent w/ above?
            img = np.zeros((320,240))
            
        # TODO is this unreachable now?
        if img is None:
            return
        
        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'],
                             self.params['roi_l']:self.params['roi_r']]

        # (height, width)
        self.shapeImage = self.imgScaled.shape
        
        if self.backgroundImage is not None:
            if self.backgroundImage.shape != self.imgScaled.shape:
                # TODO why would this happen in one run?
                # i might be in favor of an error here?
                self.backgroundImage = None
                self.reset_background_flag = True
        
        ########### Call to image processing function ##########################
        # must be defined seperately - see "main" code at the bottom of this
        # script
        self.process_image()
        ########################################################################
        
    def main(self):
        while (not rospy.is_shutdown()):
            t = rospy.Time.now().to_sec() - self.time_start
            if t > self.record_length_seconds:
                return
            with self.lockBuffer:
                # TODO TODO only calc time_now and pt if buffer is in warning
                # regime?
                time_now = rospy.Time.now()
                if len(self.image_buffer) > 0:
                    self.process_image_buffer(self.image_buffer.pop(0))

                pt = (rospy.Time.now() - time_now).to_sec()

                if len(self.image_buffer) > 9:
                    rospy.logwarn('Tracking processing time exceeds ' + 
                        'acquisition rate. Processing time: %f, Buffer: %d',
                        pt, len(self.image_buffer))

        cv2.destroyAllWindows()


if __name__ == '__main__':
    tracker_node_basename = 'multi_tracker/tracker'

    image_processing_function = \
        rospy.get_param(tracker_node_basename + '/image_processor')

    image_processing_module = \
        rospy.get_param(tracker_node_basename + '/image_processing_module')
    
    if image_processing_module == 'default':
        catkin_node_directory = os.path.dirname(os.path.realpath(__file__))
        image_processing_module = \
            os.path.join(catkin_node_directory, 'image_processing.py')

    # put behind debug flags
    print('looking for image_processing module: ' + image_processing_module)
    print('trying to load: ' + image_processing_function)

    image_processing = \
        imp.load_source('image_processing', image_processing_module)
    image_processor = \
        image_processing.__getattribute__(image_processing_function)
    Tracker.process_image = image_processor
     
    tracker = Tracker()
    tracker.main()
