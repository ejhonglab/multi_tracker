#!/usr/bin/env python

from __future__ import division
import rospy
import copy
import numpy as np
import cv2
import dynamic_reconfigure.server
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from multi_tracker.msg import Contourinfo, Contourlist

import time, os
from distutils.version import LooseVersion, StrictVersion

print 'Using open cv: ' + cv2.__version__

# video would not load before installing most recent version of pyqtgraph from github repo
# this is the version of the commit that fixed the
# issue with current numpy: pyqtgraph-0.9.10-118-ge495bbc (in commit e495bbc...)
# version checking with distutils.version. See: http://stackoverflow.com/questions/11887762/compare-version-strings
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print 'Open CV 3'
else:
    OPENCV_VERSION = 2
    print 'Open CV 2'

    
###############################################################################
# basic image processing function (self contained), to demonstrate
# the format custom image processing functions should follow
###############################################################################
# TODO fix this if i broke it
"""
def incredibly_basic(self):
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        self.reset_background()
        return
    if self.reset_background_flag:
        self.reset_background()
        self.reset_background_flag = False
        return
      
    self.absdiff = cv2.absdiff(np.float32(self.imgScaled), self.backgroundImage)
    self.imgproc = copy.copy(self.imgScaled)
    retval, self.threshed = cv2.threshold(self.absdiff, self.params['threshold'], 255, 0)
    
    # convert to gray if necessary
    if len(self.threshed.shape) == 3:
        self.threshed = np.uint8(cv2.cvtColor(self.threshed, cv2.COLOR_BGR2GRAY))
    
    # extract and publish contours
    # http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
    if OPENCV_VERSION == 2:
        contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    elif OPENCV_VERSION == 3:
        self.threshed = np.uint8(self.threshed)
        image, contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    try:
        header  = Header(stamp=self.framestamp,frame_id=str(self.framenumber))
    except:
        header  = Header(stamp=None,frame_id=str(self.framenumber))
        rospy.logerr('could not get framestamp, run tracker_nobuffer instead')
    # TODO does floris have a tracker_nobuffer? what did it do?

    contour_info = []
    for contour in contours:
        # TODO pull request this. contour fitting should work w/ > 5 points. is CHAIN_APPROX_NONE
        # still around? or using simple? check # points
        if len(contour) >= 5: # Large objects are approximated by an ellipse
            ellipse = cv2.fitEllipse(contour)
            (x,y), (a,b), angle = ellipse
            a /= 2.
            b /= 2.
            ecc = np.min((a,b)) / np.max((a,b))
            area = np.pi*a*b
            
            data = Contourinfo()
            data.header  = header
            # TODO what happens if this is set incorrectly? detect incorrect if not using simtime?
            data.dt      = dtCamera
            data.x       = x
            data.y       = y
            data.area    = area
            data.angle   = angle
            data.ecc     = ecc
            
            contour_info.append(data)
            
        else: # Small ones get ignored
            pass
            
    # publish the contours
    self.pubContours.publish(Contourlist(header=header, contours=contour_info))  
"""


###############################################################################
# General use functions
###############################################################################

def is_point_below_line(point, slope, intercept):
    x = point[0]
    y = point[1]
    result = y-slope*x-intercept
    if result > 0:
        return False
    else:
        return True

    
def fit_ellipse_to_contour(self, contour):
    ellipse = cv2.fitEllipse(contour)
    (x,y), (a,b), angle = ellipse
    a /= 2.
    b /= 2.
    ecc = np.min((a,b)) / np.max((a,b))
    area = np.pi*a*b
    if self.params['use_moments']: # inefficient - double calculating. Is ecc use somewhere else? If not, the ellipse is not at all needed
        try:
            moments = get_centroid_from_moments(contour) # get these values from moments - might be more robust?
        except:
            moments = None
        if moments is not None:
            x, y, area = moments
    return x, y, ecc, area, angle
    

def get_centroid_from_moments(contour):
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        area = cv2.contourArea(contour)
        return cx, cy, area
    else:
        return None
        

def add_data_to_contour_info(x,y,ecc,area,angle,dtCamera,header):
    # Prepare to publish the contour info
    # contour message info: dt, x, y, angle, area, ecc
    data = Contourinfo()
    data.header  = header
    data.dt      = dtCamera
    data.x       = x
    data.y       = y
    data.area    = area
    data.angle   = angle
    data.ecc     = ecc
    return data
    

def extract_and_publish_contours(self):
    if OPENCV_VERSION == 2:
        contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    elif OPENCV_VERSION == 3:
        image, contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
    
    try:
        header  = Header(stamp=self.framestamp,frame_id=str(self.framenumber))
    except:
        header  = Header(stamp=None,frame_id=str(self.framenumber))
        print 'could not get framestamp, run tracker_nobuffer instead'
        
    contour_info = []
    coords_and_area = set()
    for contour in contours:
        # Large objects are approximated by an ellipse
        # TODO break fitting into func and make recursive?
        if len(contour) >= 5:
            x, y, ecc, area, angle = fit_ellipse_to_contour(self, contour)
            coords_and_area.add((x, y, area))
            
            # if object is too large, split it in two, this helps with colliding objects, but is not 100% correct
            if area > self.params['max_expected_area']:
                slope = np.tan(angle)
                intercept = y - slope*x
                c1 = []
                c2 = []
                for point in contour:
                    point = point.reshape(2)
                    if is_point_below_line(point, slope, intercept):
                        c1.append([point])
                    else:
                        c2.append([point])
                c1 = np.array(c1)
                c2 = np.array(c2)
                
                if len(c1) >= 5:
                    x, y, ecc, area, angle = fit_ellipse_to_contour(self, np.array(c1))
                    if area < self.params['max_size'] and area > self.params['min_size']:
                        data = add_data_to_contour_info(x,y,ecc,area,angle,self.dtCamera,header)
                        contour_info.append(data)
                
                if len(c2) >= 5:
                    x, y, ecc, area, angle = fit_ellipse_to_contour(self, np.array(c2))
                    if area < self.params['max_size'] and area > self.params['min_size']:
                        data = add_data_to_contour_info(x, y, ecc , area, angle, \
                            self.dtCamera, header)
                        contour_info.append(data)
            else:
                if area < self.params['max_size'] and area > self.params['min_size']:
                    data = add_data_to_contour_info(x,y,ecc,area,angle,self.dtCamera,header)
                    contour_info.append(data)
            
            
        # Small ones just get a point
        else:
            area = 0
            
    # publish the contours
    self.pubContours.publish( Contourlist(header = header, contours=contour_info) )  
    return coords_and_area


def convert_to_gray_if_necessary(self):
    if len(self.threshed.shape) == 3:
        self.threshed = np.uint8(cv2.cvtColor(self.threshed, cv2.COLOR_BGR2GRAY))

        
def erode_and_dialate(self):
    # TODO break this out into a param + default?
    kernel = np.ones((3,3), np.uint8)

    self.threshed = cv2.dilate(self.threshed, kernel, iterations=self.params['dilate'])
    if self.debug and self.pub_dilated.get_num_connections() > 0:
        c = cv2.cvtColor(np.uint8(self.threshed), cv2.COLOR_GRAY2BGR)
        img = self.cvbridge.cv2_to_imgmsg(c, 'bgr8') # might need to change to bgr for color cameras
        self.pub_dilated.publish(img)

    self.threshed = cv2.erode(self.threshed, kernel, iterations=self.params['erode'])
    if self.debug and self.pub_eroded.get_num_connections() > 0:
        c = cv2.cvtColor(np.uint8(self.threshed), cv2.COLOR_GRAY2BGR)
        img = self.cvbridge.cv2_to_imgmsg(c, 'bgr8') # might need to change to bgr for color cameras
        self.pub_eroded.publish(img)
    

def reset_background_if_difference_is_very_large(self, color='dark'):
    # if the fraction of changed pixels (in direction of interest)    # is above threshold, reset the background
    if color == 'dark' and np.sum(self.threshed>0) / (self.shapeImage[0] * \
        self.shapeImage[1]) > self.params['max_change_in_frame']:
        reset_background(self)
        
    elif color == 'light':
        raise NotImplementedError


def save_png(self):
    # TODO share with other place that generates these
    # is this the timestamp i want to use? maybe just count from 0? count time from 0?
    # TODO don't use ROStime if that param isn't set
    background_img_filename = self.experiment_basename + \
        time.strftime('_deltavideo_bgimg_%Y%m%d_%H%M_N' + str(self.pipeline_num) + '.png', time.localtime(rospy.Time.now().to_sec()))
    
    if self.explicit_directories:
        data_directory = os.path.expanduser(rospy.get_param('multi_tracker/data_directory'))
    else:
        data_directory = os.path.join(os.getcwd(), self.experiment_basename)
    filename = os.path.join(data_directory, background_img_filename)
    
    if self.save_data:
        try:
            # TODO TODO but are these saved images actually used?
            # when would they be loaded if not referenced in that
            # field of a DeltaVid message in the decompressor??
            # anything else loading the bgimgs?
            cv2.imwrite(filename, self.backgroundImage) # requires opencv > 2.4.9
            rospy.loginfo('Background reset: ' + filename)
        except:
            rospy.logerr('failed to save background image, might need opencv 2.4.9?')
    

# TODO TODO fix. saved images are likely not used.
# but i guess this is maybe only called from the live_viewer?
def reset_background(self):
    self.backgroundImage = copy.copy(np.float32(self.imgScaled))
    save_png(self)


# TODO TODO add a debug topic for the background image
def add_image_to_background(self, color='dark'):
    # why copy just for this?
    tmp_backgroundImage = copy.copy(np.float32(self.imgScaled))
    # TODO do i really want to take the max / min?
    if color == 'dark':
        self.backgroundImage = np.max([self.backgroundImage, tmp_backgroundImage], axis=0)
    elif color == 'light':
        self.backgroundImage = np.min([self.backgroundImage, tmp_backgroundImage], axis=0)
    # TODO TODO fix naming problems in analysis! (if that's all any pngs are
    # used for in this file, maybe just name the png <basename>_display_bg.png?
    self.save_png()
    
    
# TODO it seems that floris' older syntax (that i mostly got rid of in 9c8b92db) was 
# necessary to properly load these functions. why? revert all self.f( to f(self, ?
# or test all?
def dark_objects_only(self):
    dark_or_light_objects_only(self, color='dark')


def light_objects_only(self):
    dark_or_light_objects_only(self, color='light')


def dark_or_light_objects(self):
    dark_or_light_objects_only(self, color='darkorlight')


def dark_or_light_objects_only(self, color='dark'):
    if self.image_mask is None:
        fill_color = [1,1,1]
        if not self.params['circular_mask_x'] is None:
            self.image_mask = np.zeros_like(self.imgScaled)
            cv2.circle(self.image_mask, (self.params['circular_mask_x'], self.params['circular_mask_y']), int(self.params['circular_mask_r']), fill_color, -1)

        # TODO test!
        elif not self.params['roi_points'] is None:
            self.image_mask = np.zeros_like(self.imgScaled)
            hull = cv2.convexHull(np.array(self.params['roi_points'], dtype=np.int32))
            cv2.fillConvexPoly(self.image_mask, hull, fill_color) # , -1)

    if not self.image_mask is None:
        self.imgScaled = self.image_mask * self.imgScaled

    if self.debug and self.pub_mask.get_num_connections() > 0:
        c = cv2.cvtColor(np.uint8(self.image_mask), cv2.COLOR_GRAY2BGR)
        img = self.cvbridge.cv2_to_imgmsg(c, 'bgr8') # might need to change to bgr for color cameras
        self.pub_mask.publish(img)
    
    # TODO do i want to return in both of these cases? might cause more discontinuity
    # than necessary
    # If we need to reset background image, grab one, and move on to the next frame
    if self.backgroundImage is None or self.reset_background_flag:
        reset_background(self)
        self.reset_background_flag = False
        return
    
    if self.add_image_to_background_flag:
        self.add_image_to_background(color)
        self.add_image_to_background_flag = False
        return 
    
    # TODO TODO fix
    # TODO is this actually nonzero? warn if it's not?
    if self.params['backgroundupdate'] != 0:
        cv2.accumulateWeighted(np.float32(self.imgScaled), self.backgroundImage, self.params['backgroundupdate']) # this needs to be here, otherwise there's an accumulation of something in the background
    # break into functions?
    if self.params['medianbgupdateinterval'] != 0:
        t = rospy.Time.now().to_sec()
        # TODO TODO used?
        if not self.__dict__.has_key('medianbgimages'):
            self.medianbgimages = [self.imgScaled]
            self.medianbgimages_times = [t]
        if t-self.medianbgimages_times[-1] > self.params['medianbgupdateinterval']:
            self.medianbgimages.append(self.imgScaled)
            self.medianbgimages_times.append(t)
        if len(self.medianbgimages) > 3:
            self.backgroundImage = copy.copy(np.float32(np.median(self.medianbgimages, axis=0)))
            self.medianbgimages.pop(0)
            self.medianbgimages_times.pop(0)
            rospy.loginfo('reset background with median image')

    # TODO put in init?
    try:
        kernel = self.kernel
    except AttributeError:
        kern_d = self.params['morph_open_kernel_size']
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_d,kern_d))
        self.kernel = kernel
    
    if color == 'dark':
        self.threshed = cv2.compare(np.float32(self.imgScaled), self.backgroundImage-self.params['threshold'], cv2.CMP_LT) # CMP_LT is less than
    
    elif color == 'light':
        self.threshed = cv2.compare(np.float32(self.imgScaled), self.backgroundImage+self.params['threshold'], cv2.CMP_GT) # CMP_GT is greater than
    
    elif color == 'darkorlight':
        # TODO clean up
        #absdiff = cv2.absdiff(np.float32(self.imgScaled), self.backgroundImage)
        #retval, self.threshed = cv2.threshold(absdiff, self.params['threshold'], 255, 0)
        #self.threshed = np.uint8(self.threshed)
        dark = cv2.compare(np.float32(self.imgScaled), self.backgroundImage-self.params['threshold'], cv2.CMP_LT) # CMP_LT is less than
        light = cv2.compare(np.float32(self.imgScaled), self.backgroundImage+self.params['threshold'], cv2.CMP_GT) # CMP_GT is greater than
        self.threshed = dark+light
    
    convert_to_gray_if_necessary(self)

    if self.debug and self.pub_threshed.get_num_connections() > 0:
        c = cv2.cvtColor(np.uint8(self.threshed), cv2.COLOR_GRAY2BGR)
        img = self.cvbridge.cv2_to_imgmsg(c, 'bgr8') # might need to change to bgr for color cameras
        self.pub_threshed.publish(img)
    
    # noise removal
    if self.params['denoise']:
        self.threshed = cv2.morphologyEx(self.threshed,cv2.MORPH_OPEN, kernel, iterations = 1)

        if self.debug and self.pub_denoised.get_num_connections() > 0:
            c = cv2.cvtColor(np.uint8(self.threshed), cv2.COLOR_GRAY2BGR)
            img = self.cvbridge.cv2_to_imgmsg(c, 'bgr8') # might need to change to bgr for color cameras
            self.pub_denoised.publish(img)
    
    erode_and_dialate(self)

    coords_and_area = extract_and_publish_contours(self)

    # publish the processed image
    # for troubleshooting image processing pipeline
    if self.debug and self.pub_processed.get_num_connections() > 0:
        cimg = cv2.cvtColor(np.uint8(self.threshed), cv2.COLOR_GRAY2BGR)
        for x, y, area in coords_and_area:
            cv2.putText(cimg, str(area), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, \
                (255, 255, 255), 1, cv2.LINE_AA)

        # TODO test whether this works in OpenCV 2 and 3 (i think it works in 3 for me, despite
        # Floris's comment
        img = self.cvbridge.cv2_to_imgmsg(cimg, 'bgr8') # might need to change to bgr for color cameras
        self.pub_processed.publish(img)

    reset_background_if_difference_is_very_large(self, color)
        
