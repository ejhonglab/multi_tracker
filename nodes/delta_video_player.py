#!/usr/bin/env python

from __future__ import division

import copy
import imp
import os
import traceback
from optparse import OptionParser

import cv2
import rospy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image

from multi_tracker.msg import DeltaVid

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: /camera/image_raw

# for firefley cameras, camera1394 does not provide timestamps but otherwise
# works. use point grey drivers.
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: /camera/image_mono


from distutils.version import LooseVersion, StrictVersion
print 'Using open cv: ' + cv2.__version__
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print 'Open CV 3'
else:
    OPENCV_VERSION = 2
    print 'Open CV 2'

# The main tracking class, a ROS node
class DeCompressor:
    def __init__(self, topic_in, topic_out, directory, config=None, mode='mono',
                 saveto='', fps=5.0):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # initialize the node
        rospy.init_node('delta_decompressor')
        
        # Publishers - publish contours
        self.pubDeltaVid = rospy.Publisher(topic_out, Image, queue_size=30)
        self.subDeltaVid = rospy.Subscriber(topic_in, DeltaVid,
            self.delta_image_callback, queue_size=30)
        
        self.cvbridge = CvBridge()

        #rospy.get_param('multi_tracker/delta_video/directory', default='')
        self.directory = directory 
        self.backgroundImage = None
        self.background_img_filename = 'none'
        
        self.config = config
        self.mode = mode
        
        if len(saveto) > 0:
            self.saveto = saveto
            self.videowriter = None
        else:
            self.saveto = None
            self.videowriter = None

        self.fps = fps
            
        
    def delta_image_callback(self, delta_vid):
        if (self.background_img_filename != delta_vid.background_image or
            self.backgroundImage is None):

            self.background_img_filename = delta_vid.background_image
            basename = os.path.basename(self.background_img_filename)
            directory_with_basename = os.path.join(self.directory, basename)
            
            try:
                if not os.path.exists(directory_with_basename):
                    raise IOError('background image file ' +
                                  directory_with_basename + ' did not exist')

                if not os.path.getsize(directory_with_basename) > 0:
                    raise IOError('background image file ' +
                                  directory_with_basename + ' was empty')
            
            except IOError:
                traceback.print_exc()
                # this (should) just shutdown the current node, which can be
                # marked as required in the launch file (bringing everything
                # down if it goes down)
                rospy.signal_shutdown(
                    'cannot proceed without background images.')
            
            self.backgroundImage = cv2.imread(directory_with_basename,
                                              cv2.CV_8UC1)
            try:
                # for hydro
                self.backgroundImage = self.backgroundImage.reshape(
                    [self.backgroundImage.shape[0], self.backgroundImage[1], 1])

            # TODO handle cases by version explicitly or at least specify
            # expected error
            except:
                # for indigo
                pass 
                
        if self.backgroundImage is not None:
            new_image = copy.copy(self.backgroundImage)
            
            if delta_vid.values is not None:
                if len(delta_vid.values) > 0:
                    try:
                        # for hydro
                        new_image[delta_vid.xpixels, delta_vid.ypixels, 0] = \
                            delta_vid.values
                    except:
                        # for indigo
                        new_image[delta_vid.xpixels, delta_vid.ypixels] = \
                            delta_vid.values
            
            if self.mode == 'color':
                new_image = cv2.cvtColor(new_image, cv2.COLOR_GRAY2RGB)

            if self.config is not None:
                # just use ros time conversion func
                t = (delta_vid.header.stamp.secs +
                     delta_vid.header.stamp.nsecs * 1e-9)
                self.config.draw(new_image, t)
                
            if self.saveto is not None:
                # TODO why not move this to init?
                if self.videowriter is not None:
                    self.videowriter.write(new_image)
                else:
                    '''
                    if OPENCV_VERSION == 2:
                        # works on Linux and Windows
                        self.videowriter = cv2.VideoWriter(self.saveto,
                            cv2.cv.CV_FOURCC('m','p','4','v'), 300,
                            (new_image.shape[1], new_image.shape[0]), True)

                    elif OPENCV_VERSION == 3:
                        self.videowriter = cv2.VideoWriter(self.saveto,
                            cv2.VideoWriter_fourcc('m','p','4','v'), 300,
                            (new_image.shape[1], new_image.shape[0]), True)
                    '''
                    # TODO handle iscolor flag appropriately
                    self.videowriter = cv2.VideoWriter(self.saveto, \
                        cv2.VideoWriter_fourcc(*'XVID'), self.fps, \
                        (new_image.shape[1], new_image.shape[0]), False)
                    #self.videowriter.open(self.saveto,
                    #   cv.CV_FOURCC('P','I','M','1'), 30,
                    #   (new_image.shape[0], new_image.shape[1]))

            if self.mode == 'mono':
                image_message = self.cvbridge.cv2_to_imgmsg(new_image,
                                                            encoding="mono8")
            elif self.mode == 'color':
                image_message = self.cvbridge.cv2_to_imgmsg(new_image,
                                                            encoding="bgr8")
            
            image_message.header = delta_vid.header
            self.pubDeltaVid.publish(image_message)
            
    
    def main(self):
        rospy.spin()
        if self.videowriter is not None:
            self.videowriter.release()
            print ('Note: use this command to make a mac / quicktime friendly' +
                   'video: avconv -i test.avi -c:v libx264 -c:a copy ' +
                   'outputfile.mp4')


if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("--in", type="str", dest="input",
        default='multi_tracker/delta_video', help="input topic name")

    parser.add_option("--out", type="str", dest="output",
        default='camera/image_decompressed', help="output topic name")

    parser.add_option("--directory", type="str", dest="directory",
        default=os.getcwd(),
        help="directory where background images can be found")

    # TODO --draw or something less generic than config? previous uses?
    parser.add_option("--config", type="str", dest="config", default='',
        help=("configuration file, which should describe a class " +
              "that has a method draw"))

    parser.add_option("--mode", type="str", dest="mode", default='mono',
                      help="color if desired to convert to color image")

    parser.add_option("--saveto", type="str", dest="saveto", default='',
        help=("filename where to save video, default is none. Note: use this" +
            "command to make a mac / quicktime friendly video: avconv -i " +
            "test.avi -c:v libx264 -c:a copy outputfile.mp4"))

    # TODO add fps option

    (options, args) = parser.parse_args()

    if len(options.config) > 0: 
        config = imp.load_source('config', options.config)
        c = config.Config(options.config)
    else:
        c = None
        
    decompressor = DeCompressor(options.input, options.output,
        options.directory, c, options.mode, options.saveto)

    decompressor.main()
