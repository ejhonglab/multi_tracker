#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import rosparam
import rosnode
import copy
import numpy as np
import cv2
import threading
import dynamic_reconfigure.server
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header, String

from multi_tracker.msg import Contourinfo, Contourlist
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import addImageToBackgroundService

import image_processing

from distutils.version import LooseVersion, StrictVersion
print 'Using open cv: ' + cv2.__version__
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print 'Open CV 3'
else:
    OPENCV_VERSION = 2
    print 'Open CV 2'

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: /camera/image_raw

# for firefley cameras, camera1394 does not provide timestamps but otherwise works
# use point grey drivers
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: /camera/image_mono

# Trajectory class to aid in drawing colored tracked trajectories with opencv
class Trajectory(object):
    def __init__(self, objid):
        self.objid = objid
        self.positions = []
        self.covariances = []
        self.color = np.random.randint(0,255,3).tolist()
        self.popout = False

def draw_trajectory(img, pts, color, thickness):
    for i in range(len(pts)-3):
        try:
            cv2.line(img, (int(pts[i][0]), int(pts[i][1])), (int(pts[i+1][0]), int(pts[i+1][1])), color, thickness)
        except:
            rospy.logwarn('could not draw trajectory line, length pts: ' + \
                str(len(pts)) + 'i: ' + str(i))


# The main tracking class, a ROS node
class LiveViewer:
    def __init__(self):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
        self.params = { 'image_topic'               : 'camera/image_mono',
                        'min_persistence_to_draw'   : 10,
                        'max_frames_to_draw'        : 50,
                        'camera_encoding'           : 'mono8', # fireflies are bgr8, basler gige cams are mono8
                        'roi_l'                     : 0,
                        'roi_r'                     : -1,
                        'roi_b'                     : 0,
                        'roi_t'                     : -1,
                        '~circular_mask_x'           : None,
                        '~circular_mask_y'           : None,
                        '~circular_mask_r'           : None,
                        '~roi_points'                : None,
                        '~detect_tracking_pipelines' : False, # rename?
                        'save_demo'                  : False
                        }
        
        for parameter, default_value in self.params.items():
            use_default = False
            try:
                if parameter[0] == '~':
                    value = rospy.get_param(parameter)
                else:
                    try:
                        value = rospy.get_param('multi_tracker/liveviewer/' + parameter)
                    except KeyError:
                        value = rospy.get_param('multi_tracker/tracker/' + parameter)
                
                # for maintaining backwards compatibility w/ Floris' config files that
                # would use 'none' to signal default should be used.
                # may break some future use cases.
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

        self.videowriter = None
        if self.params['save_demo']:
            # TODO include timestamp?
            self.video_filename = 'tracking_demo.avi'
            self.desired_frame_rate = 30.0
            self.mode = 'color'

        rospy.init_node('liveviewer')
        

        self.tracked_trajectories = {}
        self.clear_rois()

        # should i allow both roi_* and detect_tracking_pipelines?
        roi_params = ['circular_mask_x', 'circular_mask_y', 'circular_mask_r', 'roi_points']
        if self.params['detect_tracking_pipelines']:
            if any(map(lambda x: self.params[x] != None, roi_params)):
                rospy.logfatal('liveviewer: roi parameters other than rectangular roi_[l/r/b/t] ' + \
                    'are not supported when detect_tracking_pipelines is set to True')

        # add single rois defined in params to instance variables for consistency later
        # roi_* are still handled differently, and can be set alongside tracker roi_*'s
        # which are now private (node specific) parameters
        else:
            n = rospy.get_name()
            try:
                node_num = int(n.split('_')[-1])
            except ValueError:
                node_num = 1
            
            circle_param_names = ['circular_mask_x', 'circular_mask_y', 'circular_mask_r'] 
            self.add_roi(node_num, circle_param_names)

            poly_param_names = ['roi_points']
            # TODO break roi stuff and subscription initiation
            # into one function?
            self.add_roi(node_num, poly_param_names)
            
            # otherwise, we need to start subscriptions as we
            # detect other pipelines in our namespace
            self.start_subscribers(node_num)

        # TODO put in dict above? (& similar lines in other files)
        # displays extra information about trajectory predictions / associations if True
        self.debug = rospy.get_param('multi_tracker/liveviewer/debug', False)
        
        # initialize display
        self.window_name = 'liveviewer'
        self.cvbridge = CvBridge()
        if self.params['detect_tracking_pipelines']:
            self.contours = dict()
        else:
            self.contours = None
        self.window_initiated = False
        self.image_mask = None 
        
        # Subscriptions - subscribe to images, and tracked objects
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subImage = rospy.Subscriber(self.params['image_topic'], Image, self.image_callback, queue_size=5, buff_size=2*sizeImage, tcp_nodelay=True)

        # for adding images to background
        add_image_to_background_service_name = 'multi_tracker/tracker/add_image_to_background'
        rospy.wait_for_service(add_image_to_background_service_name)
        try:
            self.add_image_to_background = rospy.ServiceProxy(add_image_to_background_service_name, addImageToBackgroundService)
        
        except:
            rospy.logerr('could not connect to add image to background service - is tracker running?')

        

    # TODO refactor to not have node_num optional, and that signal whether num should be passed to callback?
    def start_subscribers(self, node_num):
        self.tracked_trajectories[node_num] = dict()
        rospy.Subscriber('multi_tracker/tracked_objects_' + str(node_num), Trackedobjectlist, \
            lambda x: self.tracked_object_callback(node_num, x))
        # TODO refactor to not special case as much with the none. just make sure dict w/ one entry
        # publishes to correct topic
        if self.params['detect_tracking_pipelines']:
            rospy.Subscriber('multi_tracker/contours_' + str(node_num), Contourlist, \
                lambda x: self.contour_callback(x, n=node_num))
        else:
            rospy.Subscriber('multi_tracker/contours_' + str(node_num), Contourlist, \
                self.contour_callback)
    

    def reset_background(self, service_call):
        self.reset_background_flag = True
        return 1
        
    
    def tracked_object_callback(self, node_num, tracked_objects):
        for trajec in self.tracked_trajectories[node_num].values():
            trajec.popout = True
    
        # TODO is tracked_objects.tracked_objects correct? why?
        for tracked_object in tracked_objects.tracked_objects:
            if tracked_object.persistence > self.params['min_persistence_to_draw']:
                # create new object
                if tracked_object.objid not in self.tracked_trajectories[node_num].keys():
                    self.tracked_trajectories[node_num][tracked_object.objid] = Trajectory(tracked_object.objid)
                
                # update tracked objects
                # TODO why multiple covariances? how used?
                self.tracked_trajectories[node_num][tracked_object.objid]\
                    .covariances.append(tracked_object.covariance)
                self.tracked_trajectories[node_num][tracked_object.objid].positions.append(\
                    [tracked_object.position.x, tracked_object.position.y])
                
                # if it is a young object, let it grow to max length
                if len(self.tracked_trajectories[node_num][tracked_object.objid].positions) \
                    < self.params['max_frames_to_draw']:
                    
                    self.tracked_trajectories[node_num][tracked_object.objid].popout = False
        
        # cull old objects
        for objid, trajec in self.tracked_trajectories[node_num].items():
            if trajec.popout:
                trajec.positions.pop(0)
                trajec.covariances.pop(0)
                if len(trajec.positions) <= 1:
                    del(self.tracked_trajectories[node_num][objid])

    
    def contour_callback(self, contours, n=None):
        if n is None:
            self.contours = contours
        else:
            self.contours[n] = contours


    def image_callback(self, rosimg):
        # Convert the image.
        try:
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
            # TODO why do i want this? delete?
            img = np.zeros((320,240))
        
        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'], self.params['roi_l']:self.params['roi_r']]
        self.shapeImage = self.imgScaled.shape # (height,width)

        # TODO reformat old roi params in this format in __init__?
        # TODO TODO will need to call this when self.image_mask is none, or when we add a new
        # roi (might just reset image_mask to none if thats the case)
        # TODO what about if rois go down?
        # TODO maybe hold masks for each roi, and union in a separate step? / remove / add masks
        # from another instance variable when roi definitions are removed / added?
        
        # make a mask that is a union of all of the ROIs we are tracking
        # if we have any rois
        if self.have_rois:
            if self.image_mask is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                fill_color = [1,1,1]
                
                for r in self.circular_rois.values():
                    cv2.circle(self.image_mask, (r['circular_mask_x'], r['circular_mask_y']), \
                        int(r['circular_mask_r']), fill_color, -1) # need to cast? TODO

                # TODO maybe these should not be added as one element lists, like they seem to be?
                for r in self.polygonal_rois.values():
                    cv2.fillConvexPoly(self.image_mask, r, fill_color)

                for r in self.rectangular_rois:
                    # TODO correct? shape?
                    self.image_mask[r['roi_b']:r['roi_t'], r['roi_l']:r['roi_r']] = fill_color
            self.imgScaled = self.image_mask*self.imgScaled
        
        # Image for display
        if self.params['camera_encoding'] == 'mono8':
            self.imgOutput = cv2.cvtColor(self.imgScaled, cv2.COLOR_GRAY2RGB)
        
        elif self.params['camera_encoding'] == 'binary':
            self.imgOutput = self.imgScaled
        
        else:
            self.imgOutput = self.imgScaled

        if self.have_rois and self.params['detect_tracking_pipelines']:
            # TODO implement for other roi types as well
            for n, r in self.polygonal_rois.items():
                text_center = (int(np.mean(map(lambda x: x[0], r[0]))) - 25, \
                    min(map(lambda x: x[1], r[0])) - 185)
                cv2.putText(self.imgOutput, str(n), text_center, \
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Draw ellipses from contours
        # TODO consider just using iterable identity?
        if self.contours is not None:
            if self.params['detect_tracking_pipelines']:
                contours = self.contours.values()
            else:
                contours = [self.contours]

            for contour_list in contours:
                for c, contour in enumerate(contour_list.contours):
                    # b = contour.area / (np.pi*a)
                    # b = ecc*a
                    if contour.ecc != 0: # eccentricity of ellipse < 1 but > 0
                        a = np.sqrt( contour.area / (np.pi*contour.ecc) )
                        b = contour.ecc*a
                    else: # eccentricity of circle is 0 
                        a = 1
                        b = 1
                    center = (int(contour.x), int(contour.y))
                    angle = int(contour.angle)
                    axes = (int(np.min([a,b])), int(np.max([a,b])))
                    cv2.ellipse(self.imgOutput, center, axes, angle, 0, 360, (0,255,0), 2 )
                    
                    if self.debug:
                        # assumes consistent order in contourlist, read in same iteration as data_assoc,
                        # and that data_assoc doesn't change order
                        offset_center = (int(contour.x) + 15, int(contour.y))
                        cv2.putText(self.imgOutput, str(c), offset_center, cv2.FONT_HERSHEY_SIMPLEX, \
                            0.65, (0,255,0), 2, cv2.LINE_AA)
        
        # Display the image | Draw the tracked trajectories
        for pipeline_num in self.tracked_trajectories:
            for objid, trajec in self.tracked_trajectories[pipeline_num].items():
                if len(trajec.positions) > 5:
                    draw_trajectory(self.imgOutput, trajec.positions, trajec.color, 2)
                    trajec_center = (int(trajec.positions[-1][0]), int(trajec.positions[-1][1]))
                    cv2.circle(self.imgOutput, trajec_center, int(trajec.covariances[-1]), \
                        trajec.color, 2)
                    
                    if self.debug:
                        offset_center = (int(trajec.positions[-1][0]) - 45, int(trajec.positions[-1][1]))
                        cv2.putText(self.imgOutput, str(objid), offset_center, \
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, trajec.color, 2, cv2.LINE_AA)
                    '''
                    #if self.debug:
                        # TODO display predictions in same color (x w/ dashed line?)
                        # should i just display pos + velocity * (frame / prediction) interval?
                        # or also something about weight it will have? which kalman parameter?
                        
                        # TODO display (sorted? cost listed?) associations between trajectories 
                        # and contours
                    '''

        # to show images bigger than the screen resolution
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(self.window_name, self.imgOutput)

        if self.videowriter is not None:
            self.videowriter.write(self.imgOutput)

        elif self.params['save_demo']:
            format_code = 'XVID'
            if OPENCV_VERSION == 2:
                fourcc = cv2.cv.CV_FOURCC(*format_code)
                self.videowriter = cv2.VideoWriter(self.video_filename, fourcc, \
                    self.desired_frame_rate, (self.imgOutput.shape[1], self.imgOutput.shape[0]), \
                    1 if self.mode == 'color' else 0)

            elif OPENCV_VERSION == 3:
                fourcc = cv2.VideoWriter_fourcc(*format_code)
                self.videowriter = cv2.VideoWriter(self.video_filename, fourcc, \
                    self.desired_frame_rate, (self.imgOutput.shape[1], self.imgOutput.shape[0]), \
                    1 if self.mode == 'color' else 0)

        if not self.window_initiated: # for some reason this approach works in opencv 3 instead of previous approach
            cv2.setMouseCallback(self.window_name, self.on_mouse_click)
            self.window_initiated = True
        
        ascii_key = cv2.waitKey(1)
        if ascii_key != -1:
            self.on_key_press(ascii_key)
        

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            rospy.loginfo('clicked pixel: ' + str([x, y]))
    

    def on_key_press(self, ascii_key):
        key = chr(ascii_key)
        if key == 'a':
            resp = self.add_image_to_background()
            rospy.loginfo('added imaging to background (on keypress in liveviewer)')

    
    # TODO make less convoluted
    def add_roi(self, node_num, param_names, get_fn=None):
        r = dict()
        if get_fn is None:
            have_params = list(map(lambda x: self.params[x] != None, param_names))
            if any(have_params):
                if all(have_params):
                    for p in param_names:
                        r[p] = self.params[p]
                else:
                    rospy.logfatal('liveviewer: incomplete definition of roi type. ' + \
                        'need all of : ' + str(param_names))
            else:
                # TODO why was this causing problems in delta_video but not here?
                # is this correct? (i think so)
                return
            
        else:
            # TODO lead to same error either way...
            for p in param_names:
                r[p] = get_fn(p)
        
        if 'roi_points' in param_names:
            hull = cv2.convexHull(np.array(r['roi_points'], dtype=np.int32))
            self.polygonal_rois[node_num] = hull
        
        elif 'circular_mask_x' in param_names:
            self.circular_rois[node_num] = r

        elif 'roi_l' in param_names:
            self.rectangular_rois[node_num] = r


    def clear_rois(self):
        """
        Does not clear mask.
        """
        self.have_rois = False
        self.circular_rois = dict()
        self.polygonal_rois = dict()
        self.rectangular_rois = dict()
    
    
    def add_any_pipeline_rois(self):
        # TODO does the namespace argument do what i want?
        ns = rospy.get_namespace()
        nodes = rosnode.get_node_names(namespace=ns)
        # TODO delete me
        #rospy.logwarn('nodes w/ namespace=' + ns + ':' + str(nodes))
        #rospy.logwarn('nodes w/o namespace specified:' + str(rosnode.get_node_names()))

        roi_params = [['circular_mask_x', 'circular_mask_y', 'circular_mask_r'], \
            ['roi_points'], ['roi_l', 'roi_r', 'roi_b', 'roi_t']]

        new_nodes = False
        for n in nodes:
            if 'tracker_' in n:
                def getter(p):
                    return rospy.get_param(n + '/' + p)
            else:
                continue
            node_num = int(n.split('_')[-1])
            
            if not any(map(lambda d: node_num in d, [self.circular_rois, self.polygonal_rois, \
                self.rectangular_rois])):
                new_nodes = True
                break

        if not new_nodes:
            return

        self.clear_rois()

        for n in nodes:
            if 'tracker_'  in n:
                def getter(p):
                    return rospy.get_param(n + '/' + p)
            else:
                continue
            
            for params in roi_params:
                # may want to replace with rosnode API instrospection at some point
                # provided it is stable / well documented enough
                try:
                    # TODO need to prepend namespace?
                    # TODO TODO fix whatever causing roi_points get to be unreachable
                    # that is the one that is set, but it just tries for roi_l!
                    node_num = int(n.split('_')[-1])
                    self.add_roi(node_num, params, get_fn=getter)
                    # if the above roi is added successfully
                    # we will make it to this line, otherwise
                    # we will end up in the except block
                    self.start_subscribers(node_num)
                     
                except KeyError:
                    pass
        
        if any(map(lambda x: len(x) > 0, [self.circular_rois, self.polygonal_rois, \
            self.rectangular_rois])):
            self.have_rois = True

        else:
            self.have_rois = False

        # TODO TODO do some caching / checking to not have to recompute this everytime
        self.image_mask = None
    
    
    def main(self):
        check_for_rois_rate = rospy.Rate(10)
        if self.params['detect_tracking_pipelines']:
            while not rospy.is_shutdown():
                self.add_any_pipeline_rois()
                check_for_rois_rate.sleep()
        else:
            rospy.spin()
        cv2.destroyAllWindows()

#####################################################################################################
    
if __name__ == '__main__':
    liveviewer = LiveViewer()
    liveviewer.main()
