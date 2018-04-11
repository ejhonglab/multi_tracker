#!/usr/bin/env python

from __future__ import division

import copy
import threading
from subprocess import Popen
import time
import os
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import rospy
import rosparam
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header, String

from multi_tracker.msg import Contourinfo, Contourlist, DeltaVid
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService, RegisterROIs
import image_processing

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: /camera/image_raw

# for firefley cameras, camera1394 does not provide timestamps but otherwise
# works. use point grey drivers.
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: /camera/image_mono

# TODO have everything masked when ~wait_for_rois is true
# until rois are registered
# TODO TODO show mask for debugging

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

        self.child = None
        
        # default parameters (parameter server overides them)
        # TODO set via default yaml?
        # share this default setting code?
        # TODO does camera need to be global if multiple multi_trackers are
        # going to operate on it?  idiomatic way to do that? is what floris did
        # the best?
        self.params = {
            'image_topic'        : 'camera/image_raw',
            'threshold'          : 10,
            # fireflies are bgr8, basler gige cams are mono8
            'camera_encoding'    : 'mono8',
            'max_change_in_frame': 0.2,
            'roi_l'              : 0,
            'roi_r'              : -1,
            'roi_b'              : 0,
            'roi_t'              : -1,
            '~circular_mask_x'   : None,
            '~circular_mask_y'   : None,
            '~circular_mask_r'   : None,
            '~roi_points'        : None,
            # TODO TODO implement
            '~wait_for_rois'     : False
        }

        for parameter, default_value in self.params.items():
            # TODO shrink this try / except so it is just around
            # get_param (and so regular self.param[key] calls
            # could also be producing this error)?
            use_default = False
            try:
                # this signifies private parameters
                if parameter[0] == '~':
                    value = rospy.get_param(parameter)
                else:
                    p = 'multi_tracker/delta_video/' + parameter
                    value = rospy.get_param(p)

                # assumes strings like 'none' floris used
                # should not be overwriting defaults of None.
                # may not always be true.
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

        self.clear_rois()

        # TODO TODO share in utility module w/ liveviewer somehow
        # should i allow both roi_* and wait_for_rois?
        roi_params = ['circular_mask_x', 'circular_mask_y',
                      'circular_mask_r', 'roi_points']
        if self.params['wait_for_rois']:
            if any(map(lambda x: self.params[x] != None, roi_params)):
                rospy.logfatal('liveviewer: roi parameters other than ' + 
                    'rectangular roi_[l/r/b/t] are not supported when ' + 
                    'wait_for_rois is set to True')

        # add single rois defined in params to instance variables for
        # consistency later roi_* are still handled differently, and can be set
        # alongside tracker roi_*'s which are now private (node specific)
        # parameters
        else:
            # TODO should i write this to allow for saving each
            # roi in a separate bag? i'm leaning towards no,
            # because i'm not seeing what use cases would benefit
            # from that...
            n = rospy.get_name()
            try:
                node_num = int(n.split('_')[-1])
            except ValueError:
                node_num = 1
            
            circle_param_names = ['circular_mask_x',
                                  'circular_mask_y',
                                  'circular_mask_r'] 
            self.add_roi(node_num, circle_param_names)

            poly_param_names = ['roi_points']
            self.add_roi(node_num, poly_param_names)
        
        self.save_data = rospy.get_param('multi_tracker/delta_video/save_data',
                                         True)
        if not self.save_data:
            rospy.logwarn('delta_video not saving data! multi_tracker' + 
                '/delta_video/save_data was False')
        
        self.debug = rospy.get_param('multi_tracker/delta_video/debug', False)

        self.use_original_timestamp = rospy.get_param(
            'multi_tracker/retracking_original_timestamp', False)
        self.experiment_basename = rospy.get_param(
            'multi_tracker/experiment_basename', None)

        if self.experiment_basename is None:
            rospy.logwarn('Basenames output by different nodes in this ' +
                    'tracker run may differ!')
            self.experiment_basename = time.strftime('%Y%m%d_%H%M%S',
                time.localtime())

        self.explicit_directories = rospy.get_param(
            'multi_tracker/explicit_directories', False)
        
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
        self.pubDeltaVid = rospy.Publisher(delta_video_topic, DeltaVid,
                                           queue_size=30)

        # determine what kind of differences from background frame we are
        # interested in
        tracking_fn = rospy.get_param('multi_tracker/tracker/image_processor')
        if tracking_fn == 'dark_objects_only':
            self.sign = -1
        elif tracking_fn == 'light_objects_only':
            self.sign = 1
        else:
            # will just use absdiff if the sign of the deviations of interest
            # isn't obvious from the name of the tracking function
            self.sign = 0
        
        # background reset service
        self.reset_background_flag = False
        # TODO was this supposed to be triggered every time tracker was?
        # any reasons not to?

        # TODO TODO refactor so that all background resets go through this node?
        self.reset_background_service = rospy.Service(
            'multi_tracker/delta_video/reset_background',
            resetBackgroundService, self.reset_background)
        
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
        if not self.params['wait_for_rois']:
            sizeImage = 128+1024*1024*3 # Size of header + data.
            self.subImage = rospy.Subscriber(self.params['image_topic'], Image,
                self.image_callback, queue_size=5, buff_size=2*sizeImage,
                tcp_nodelay=True)
        
        s = rospy.Service('register_rois', RegisterROIs, self.register_rois)
        # TODO does this need a rospy.spin / spinOnce to work?


    def clear_rois(self):
        """
        Does not clear mask.
        """
        self.have_rois = False
        self.circular_rois = dict()
        self.polygonal_rois = dict()
        self.rectangular_rois = dict()


    # make less convoluted
    def add_roi(self, node_num, param_names):
        r = dict()
        have_params = list(map(lambda x: self.params[x] != None, param_names))
        #rospy.logwarn(have_params)
        if any(have_params):
            if all(have_params):
                for p in param_names:
                    #rospy.logwarn(p)
                    #rospy.logwarn(self.params[p])
                    r[p] = self.params[p]
            else:
                rospy.logfatal('liveviewer: incomplete definition of roi type' +
                    '. need all of : ' + str(param_names))

        else:
            return

        #rospy.logwarn(param_names)
        #rospy.logwarn(r)
        
        if 'roi_points' in param_names:
            hull = cv2.convexHull(np.array(r['roi_points'], dtype=np.int32))
            self.polygonal_rois[node_num] = hull
        
        elif 'circular_mask_x' in param_names:
            self.circular_rois[node_num] = r

        elif 'roi_l' in param_names:
            self.rectangular_rois[node_num] = r
    

    def reset_background(self, service_call):
        rospy.logwarn('delta video service for resetting background was ' + 
            'invoked')
        self.reset_background_flag = True
        # TODO remove?
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
            # might need to change to bgr for color cameras
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough')
        except CvBridgeError, e:
            # TODO just make this fatal?
            rospy.logerr('Exception converting background image from ROS to ' + 
                'OpenCV:  %s' % e)

        # TODO i guess i could see a case for this not being mutually exclusive
        # w/ the circular mask?
        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'], 
            self.params['roi_l']:self.params['roi_r']]

        self.shapeImage = self.imgScaled.shape # (height,width)
        
        # define a self var in init that dictates whether we should mask?
        if self.have_rois:
            if self.image_mask is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                fill_color = [1,1,1]
                
                for r in self.circular_rois.values():
                    # need to cast? TODO
                    cv2.circle(self.image_mask, (r['circular_mask_x'], 
                        r['circular_mask_y']), int(r['circular_mask_r']), 
                        fill_color, -1)

                for r in self.polygonal_rois.values():
                    cv2.fillConvexPoly(self.image_mask, r, fill_color)

                for r in self.rectangular_rois:
                    # TODO correct? shape?
                    self.image_mask[r['roi_b']:r['roi_t'], 
                                    r['roi_l']:r['roi_r']] = fill_color
            self.imgScaled = self.image_mask*self.imgScaled

        def background_png_name():
            # TODO also check we are using sim_time
            if self.use_original_timestamp:
                # TODO make sure everything that loads these doesn't break w/
                # addition of seconds
                background_img_filename = self.experiment_basename + \
                    time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M%S_N' + 
                    str(self.pipeline_num) + '.png', 
                    time.localtime(rospy.Time.now().to_sec()))
            else:
                background_img_filename = self.experiment_basename + \
                    time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M%S_N' + 
                    str(self.pipeline_num) + '.png', time.localtime())

            if self.explicit_directories:
                data_directory = os.path.expanduser(
                    rospy.get_param('multi_tracker/data_directory'))
            else:
                # TODO is this cwd really what i want?
                data_directory = os.path.join(os.getcwd(), 
                                              self.experiment_basename)
            
            return os.path.join(data_directory, background_img_filename)
        
        ### image processing function ##########################################
        
        # TODO it doesn't seem like self.current_background_img counter is used?
        # was it ever? remove?
        # If there is no background image, grab one, and move on to the next
        # frame.
        if self.backgroundImage is None:
            self.backgroundImage = copy.copy(self.imgScaled)
            self.background_img_filename = background_png_name()
            if self.save_data:
                rospy.loginfo('delta_video saving first background image to ' +
                    self.background_img_filename)

                # TODO fix offset (indices probably shifted to left, up one)
                #rospy.logwarn(self.backgroundImage)
                #rospy.logwarn(self.backgroundImage.shape)

                success = cv2.imwrite(self.background_img_filename,
                                      self.backgroundImage)

                # restarting seemed to fix this for me once, unclear why
                if (not success or not 
                    (os.path.exists(self.background_img_filename) and
                    os.path.getsize(self.background_img_filename) > 8)):
                    # TODO i'd rather do without he sys.exit call, but logfatal
                    # didn't seem to kill the node... why wouldnt it?
                    rospy.logfatal('background image png was not saved ' +
                        'correctly. reason unknown. you may consider ' +
                        'restarting.')
                    sys.exit()
                
            self.current_background_img += 1
            return

        # TODO TODO include a thread lock on reset bg flag + lock when writing?
        # TODO will this be true by default? if so, is it always saving two
        # images? maybe get rid of the above?
        if self.reset_background_flag:
            self.reset_background_flag = False
            #rospy.loginfo('resetting background')
            # put behind debug flag
            #assert not np.allclose(self.backgroundImage, self.imgScaled)
            self.backgroundImage = copy.copy(self.imgScaled)
            self.background_img_filename = background_png_name()
            
            if self.save_data:
                #rospy.loginfo('writing to ' + self.background_img_filename)
                # TODO also check for success as above
                # refactor into function
                cv2.imwrite(self.background_img_filename, self.backgroundImage)
            
            self.current_background_img += 1
            return

        # calculate the difference from the background
        # sign set in __init__ based on name of tracking function
        # (defaults to 0)
        # TODO test
	'''
        if self.sign == -1:
	    # TODO need to multiply one by -1?
            self.diff = self.backgroundImage - self.imgScaled
        elif self.sign == 1:
            self.diff = self.imgScaled - self.backgroundImage
        else:
	'''
        # TODO temporary hack fix. revert.
        self.diff = cv2.absdiff(self.imgScaled, self.backgroundImage)

        #rospy.loginfo('comparing to threshold ' +
        #    str(self.params['threshold']))
        changed_pixels = np.where(self.diff > self.params['threshold'])
        delta_msg = DeltaVid()
        header  = Header(stamp=self.framestamp,frame_id=str(self.framenumber))
        delta_msg.header = header
        delta_msg.background_image = self.background_img_filename
        
        if len(changed_pixels[0]) > 0:
            delta_msg.xpixels = changed_pixels[0].tolist()
            delta_msg.ypixels = changed_pixels[1].tolist()
            delta_msg.values = self.imgScaled[changed_pixels].reshape(
                len(changed_pixels[0])).tolist()
        else:
            # TODO why preferable for first two fields but not the values?
            delta_msg.xpixels = [0]
            delta_msg.ypixels = [0]
            #delta_msg.values = [0]
        self.pubDeltaVid.publish(delta_msg)
        
        '''
        if the fraction of the frame that changed is too large, reset the
        background
          - if this is small, it the background will reset more often, in the
            limit maybe only saving the edges of the flies
          - if this is large, the background may never reset, probably losing
            flies when they cross their original positions, and likely storing
            more data than necessary (particularly if something gets bumped or
            lighting changes)
        '''
        changed_fraction = (len(changed_pixels[0]) /
            (self.diff.shape[0] * self.diff.shape[1]))

        # TODO TODO this was printed several times but no new png. what gives?
        # flag ever effecting?
        if (not self.reset_background_flag and 
            changed_fraction > self.params['max_change_in_frame']):

            rospy.logwarn(os.path.split(__file__)[-1] + ': resetting ' + 
                'background image for # changed fraction of pixels (' + 
                str(changed_fraction) + ') > max_change_in_frame ' +
                '(' + str(self.params['max_change_in_frame']) + ')')
            self.reset_background_flag = True
            
    
    # TODO test
    # TODO TODO save rois
    def register_rois(self, req):
        """
        """
        # TODO maybe make all of this parameter setting / saving optional?

        # as of 2017-Aug, only one of lists will be non-empty.
        for i, r in enumerate(req.rectangular_rois):
            roi_dict = dict()
            roi_dict['roi_t'] = r.t
            roi_dict['roi_b'] = r.b
            roi_dict['roi_l'] = r.l
            roi_dict['roi_r'] = r.r
            self.rectangular_rois[i] = roi_dict

            # TODO does this format make the most sense?
            rospy.set_param('~' + str(i) + '/roi_t', r.t)
            rospy.set_param('~' + str(i) + '/roi_b', r.b)
            rospy.set_param('~' + str(i) + '/roi_l', r.l)
            rospy.set_param('~' + str(i) + '/roi_r', r.r)
        
        for i, r in enumerate(req.polygonal_rois):
            points = []
            for p in r.points:
                points.append([p.x, p.y])
            hull = cv2.convexHull(np.array(points, dtype=np.int32))
            self.polygonal_rois[i] = hull

            rospy.set_param('~' + str(i) + '/roi_points', points)
        
        for i, r in enumerate(req.circular_rois):
            roi_dict = dict()
            roi_dict['circular_mask_x'] = r.x
            roi_dict['circular_mask_y'] = r.y
            roi_dict['circular_mask_r'] = r.r
            self.circular_rois[i] = roi_dict

            rospy.set_param('~' + str(i) + '/circular_mask_x', r.x)
            rospy.set_param('~' + str(i) + '/circular_mask_y', r.y)
            rospy.set_param('~' + str(i) + '/circular_mask_r', r.r)

        roi_param_filename = time.strftime('compressor_rois_%Y%m%d_%H%M%S_N' +
            str(self.pipeline_num) + '.yaml', 
            time.localtime(rospy.Time.now().to_sec()))

        # now invoke the snapshot_param node in this namespace to dump the 
        # TODO maybe save through other means? just pickle? api for launching
        # single nodes?
        params = ['roslaunch', 'multi_tracker', 'snapshot_params.launch',
            'ns:=' + rospy.get_namespace(), 'filename:=' + roi_param_filename]
        self.child = Popen(params)

        self.have_rois = True
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subImage = rospy.Subscriber(self.params['image_topic'], Image,
                                         self.image_callback, queue_size=5,
                                         buff_size=2*sizeImage,
                                         tcp_nodelay=True)

        # this seems to be what ros expects for return
        # when service does not specify a return type
        return []
    
    
    def main(self):
        if self.params['wait_for_rois']:
            rate = rospy.Rate(5) # Hz:
            while not rospy.is_shutdown():
                if self.have_rois:
                    break
                rate.sleep()
        
        while not rospy.is_shutdown():
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.image_buffer) > 0:
                    self.process_image_buffer(self.image_buffer.pop(0))
                if len(self.image_buffer) > 3:
                    pt = (rospy.Time.now() - time_now).to_sec()
                    rospy.logwarn("Delta video processing time exceeds " + 
                        "acquisition rate. Processing time: %f, Buffer: %d",
                        pt, len(self.image_buffer))

        if not self.child is None:
            self.child.kill()


if __name__ == '__main__':
    compressor = Compressor()
    compressor.main()
