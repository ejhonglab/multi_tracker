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
import sys
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
        # initialize the node
        rospy.init_node('delta_compressor')
        
        # default parameters (parameter server overides them)
        # TODO set via default yaml?
        # share this default setting code?
        # TODO does camera need to be global if multiple multi_trackers are going to operate on it?
        # idiomatic way to do that? is what floris did the best?
        self.params = { 'image_topic'       : 'camera/image_raw',
                        'threshold'         : 10,
                        'camera_encoding'   : 'mono8', # fireflies are bgr8, basler gige cams are mono8
                        'max_change_in_frame'       : 0.2,
                        '~roi_l'                     : 0,
                        '~roi_r'                     : -1,
                        '~roi_b'                     : 0,
                        '~roi_t'                     : -1,
                        '~circular_mask_x'           : None,
                        '~circular_mask_y'           : None,
                        '~circular_mask_r'           : None,
                        '~roi_points'                : None
                        }

        # TODO TODO parameter alternative between rois as private parameters for each node and 
        # having delta_video_simplebuffer publishing a cropped / masked version?
        # TODO TODO i guess i already broke backwards compatibility with Floris's configuration
        # files by making these parameters private? way my version can work with Floris' original
        # configuration files?

        # possible solutions:
        # -publish private params for rois to each of subs
        # -public w/ pipeline num (pre/ap)pended?
        
        # reasons to not do either of the above: user shouldn't have to specify more than once
        # because there is no reason for them to differ across steps in pipeline
        # (except *maybe* viewer)

        # -delta_video_simplebuffer.py publish image topic to downstream tracker node, which
        #  would no longer subscribe to camera (nor need roi info)
        #    -though may need to send offset to tracker to have tracker output globally 
        #     correct coordinates?
        #  argument for this approach is the work of computing ROI masks and blanking images
        #  won't have to be done three times over.
        
        
        for parameter, default_value in self.params.items():
            # TODO shrink this try / except so it is just around
            # get_param (and so regular self.param[key] calls
            # could also be producing this error)?
            use_default = False
            try:
                # TODO do i need all the pipelines in their own namespaces regardless, to avoid topic conflicts?
                # seems like i do... (or remap all topic names or something, but this seems like what pushing down
                # was made for...)
                # this signifies private parameters
                if parameter[0] == '~':
                    value = rospy.get_param(parameter)

                else:
                    p = 'multi_tracker/delta_video/' + parameter
                    value = rospy.get_param(p)

                # TODO maybe change back to 'none' so we can specify in yamls they dont exist, while not just commenting those lines out?
                # or should i just comment the lines / not include them?
                # assumes strings like 'none' floris used
                # should not be overwriting defaults of None.
                # may not always be true.
                if self.params[parameter] is None:
                    if isinstance(value, str):
                        use_default = True

            except KeyError:
                use_default = True

            if use_default:
                rospy.loginfo(rospy.get_name() + ' using default parameter: ' + \
                    parameter + ' = ' + str(default_value))
                value = default_value
            
            if parameter[0] == '~':
                del self.params[parameter]
                parameter = parameter[1:]
            
            self.params[parameter] = value

        # TODO why does rospy.Time.now() jump from 0 to ~149998...???
        # this solution seems hacky and i wish i didn't have to do it...
        # TODO also put in other nodes? utility func?
        self.time_start = 0
        while np.isclose(self.time_start, 0.0):
            self.time_start = rospy.Time.now().to_sec()
        
        self.save_data = rospy.get_param('multi_tracker/delta_video/save_data', True)
        if not self.save_data:
            rospy.logwarn('delta_video not saving data! multi_tracker/delta_video/save_data was False')
        
        self.record_length_seconds = 3600 * rospy.get_param('multi_tracker/record_length_hours', 24)

        self.debug = rospy.get_param('multi_tracker/delta_video/debug', False)

        self.use_original_timestamp = rospy.get_param('multi_tracker/retracking_original_timestamp', False)
        self.experiment_basename = rospy.get_param('multi_tracker/experiment_basename', None)
        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this tracker run may differ!' + \
                ' Run the set_basename.py node along with others to fix this.')
            #self.experiment_basename = time.strftime('%Y%m%d_%H%M%S_N' + self.pipeline_num, time.localtime())
            self.experiment_basename = time.strftime('%Y%m%d_%H%M%S', time.localtime())

        self.explicit_directories = rospy.get_param('multi_tracker/explicit_directories', False)
        
        # TODO break into util function?
        node_name = rospy.get_name()
        last_name_component = node_name.split('_')[-1]
        try:
            self.pipeline_num = int(last_name_component)
            remap_topics = True
        
        except ValueError:
            self.pipeline_num = 1
            remap_topics = False

        delta_video_topic = 'multi_tracker/delta_video'
        
        if remap_topics:
            delta_video_topic = delta_video_topic + '_' + str(self.pipeline_num)
        
        # Publishers - publish pixel changes
        self.pubDeltaVid = rospy.Publisher(delta_video_topic, DeltaVid, queue_size=30)

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
        # TODO was this supposed to be triggered every time tracker was?
        # any reasons not to?

        # TODO TODO refactor so that all background resets go through this node?
        self.reset_background_service = rospy.Service('multi_tracker/delta_video/reset_background', resetBackgroundService, self.reset_background)
        
        self.cvbridge = CvBridge()
        self.imgScaled      = None
        self.backgroundImage = None
        self.background_img_filename = None
        
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
            # TODO warn if falling back to this? potential to cause problems?
            self.dtCamera = 0.03
        
        self.framenumber = rosimg.header.seq
        self.framestamp = rosimg.header.stamp
        
        # Convert the image.
        try:
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
            img = np.zeros((320,240))

        # TODO i guess i could see a case for this not being mutually exclusive w/ the circular mask?
        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'], self.params['roi_l']:self.params['roi_r']]
        self.shapeImage = self.imgScaled.shape # (height,width)
        
        # define a self var in init that dictates whether we should mask?
        if self.image_mask is None:
            fill_color = [1,1,1]
            if not self.params['circular_mask_x'] is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                # TODO i don't need the int cast do i?
                cv2.circle(self.image_mask, (self.params['circular_mask_x'], self.params['circular_mask_y']), int(self.params['circular_mask_r']), fill_color, -1)

            elif not self.params['roi_points'] is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                hull = cv2.convexHull(np.array(self.params['roi_points'], dtype=np.int32))
                cv2.fillConvexPoly(self.image_mask, hull, fill_color) # , -1)

        # TODO just check if key is in dict?
        elif self.params['circular_mask_x'] != None or self.params['roi_points'] != None:
            self.imgScaled = self.image_mask * self.imgScaled

        def background_png_name():
            if self.use_original_timestamp:
                background_img_filename = self.experiment_basename + \
                    time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M_N' + self.pipeline_num + \
                    '.png', time.localtime(rospy.Time.now().to_sec()))
            else:
                background_img_filename = self.experiment_basename + \
                    time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M_N' + self.pipeline_num + \
                    '.png', time.localtime())

            if self.explicit_directories:
                data_directory = os.path.expanduser(rospy.get_param('multi_tracker/data_directory'))
            else:
                data_directory = os.path.join(os.getcwd(), self.experiment_basename)
            
            return os.path.join(data_directory, background_img_filename)
        
        ### image processing function ##############################################################
        
        # TODO it doesn't seem like self.current_background_img counter is used? was it ever? remove?
        # If there is no background image, grab one, and move on to the next frame
        if self.backgroundImage is None:
            self.backgroundImage = copy.copy(self.imgScaled)
            self.background_img_filename = background_png_name()
            if self.save_data:
                rospy.loginfo('delta_video saving first background image to ' + self.background_img_filename)
                # TODO fix offset (indices probably shifted to left, up one)
                #rospy.logwarn(self.backgroundImage)
                #rospy.logwarn(self.backgroundImage.shape)

                success = cv2.imwrite(self.background_img_filename, self.backgroundImage)

                # restarting seemed to fix this for me once, unclear why
                if not success or not (os.path.exists(self.background_img_filename) and \
                    os.path.getsize(self.background_img_filename) > 8):
                    # TODO i'd rather do without he sys.exit call, but logfatal didn't seem
                    # to kill the node... why wouldnt it?
                    rospy.logfatal('background image png was not saved correctly. reason unknown. you may consider restarting.')
                    sys.exit()
                
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
            # TODO why preferable for first two fields but not the values?
            delta_msg.xpixels = [0]
            delta_msg.ypixels = [0]
            #delta_msg.values = [0]
        self.pubDeltaVid.publish(delta_msg)
        
        '''
        if the fraction of the frame that changed is too large, reset the background
          -if this is small, it the background will reset more often, in the limit maybe
           only saving the edges of the flies
          -if this is large, the background may never reset, probably losing flies when
           they cross their original positions, and likely storing more data than necessary
           (particularly if something gets bumped or lighting changes)
        '''
        changed_fraction = len(changed_pixels[0]) / (self.diff.shape[0] * self.diff.shape[1])

        # TODO TODO this was printed several times but no new png. what gives? flag ever effecting?
        if changed_fraction > self.params['max_change_in_frame']:
            rospy.logwarn(os.path.split(__file__)[-1] + ': resetting background image for # ' + \
                'changed fraction of pixels (' + str(changed_fraction) + ') > max_change_in_frame '+\
                '(' + str(self.params['max_change_in_frame']) + ')')
            self.reset_background_flag = True
            
     
    def Main(self):
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.time_start
            if t > self.record_length_seconds:
                cv2.destroyAllWindows()
                return
            
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.image_buffer) > 0:
                    self.process_image_buffer(self.image_buffer.pop(0))
                
                if len(self.image_buffer) > 3:
                    pt = (rospy.Time.now() - time_now).to_sec()
                    rospy.logwarn("Delta video processing time exceeds acquisition rate. " + \
                        "Processing time: %f, Buffer: %d", pt, len(self.image_buffer))
            
        # TODO why here?
        cv2.destroyAllWindows()

#####################################################################################################
    
if __name__ == '__main__':
    compressor = Compressor()
    compressor.Main()
