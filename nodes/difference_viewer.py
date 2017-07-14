#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser

# TODO roslib & rosparam necessary imports?
import roslib
import rospy
import rosparam
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as SensorImage

import matplotlib
matplotlib.use('TkAgg')

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure

import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk
import Image, ImageTk

class DiffViewer:
    def __init__(self, nodenum):
        # default parameters (parameter server overides them)
        self.params = { 'difference_topic'               : '/multi_tracker/' + nodenum + '/tracker/difference',
                        'camera_encoding'           : 'mono8', # fireflies are bgr8, basler gige cams are mono8
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
		# allows image processed view to be overlaid with tracked objects
                p = '/multi_tracker/' + nodenum + '/liveviewer/' + parameter
                self.params[parameter] = rospy.get_param(p)
            except:
                try:
                    p = '/multi_tracker/' + nodenum + '/tracker/' + parameter
                    self.params[parameter] = rospy.get_param(p)
                except:
                    print 'Using default parameter: ', parameter, ' = ', value
                
        # initialize the node
        rospy.init_node('diffviewer_' + nodenum)
        self.nodename = rospy.get_name().rstrip('/')
        self.nodenum = nodenum
        
        # initialize display
        self.window_name = 'difference from background'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
         
        self.cvbridge = CvBridge()
        
        # Subscriptions - subscribe to images, and tracked objects
        self.image_mask = None 
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subDiff = rospy.Subscriber('/multi_tracker/' + nodenum + '/tracker/difference', \
            SensorImage, self.diff_callback, queue_size=5, buff_size=2*sizeImage, tcp_nodelay=True)


    def diff_callback(self, rosimg):
        # Convert the image.
        try:
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting image from ROS to opencv:  %s' % e)
            return

        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'], self.params['roi_l']:self.params['roi_r']]
        self.shapeImage = self.imgScaled.shape # (height,width)
        
        if self.params['circular_mask_x'] != 'none':
            if self.image_mask is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                cv2.circle(self.image_mask,(self.params['circular_mask_x'], self.params['circular_mask_y']),int(self.params['circular_mask_r']),[1,1,1],-1)
            self.imgScaled = self.image_mask*self.imgScaled
        
        # Image for display
        if self.params['camera_encoding'] == 'mono8':
            self.imgOutput = cv2.cvtColor(self.imgScaled, cv2.COLOR_GRAY2RGB)
        elif self.params['camera_encoding'] == 'binary':
            self.imgOutput = self.imgScaled
        else:
            self.imgOutput = self.imgScaled
        
        #cv2.imshow('output', self.imgOutput)
        #ascii_key = cv2.waitKey(1)

        img = ImageTk.PhotoImage(Image.fromarray(self.imgOutput))
        Tkinter.Label(root, image=img).pack()
        print('put image in tk')
   
    '''
    def Main(self):
        while (not rospy.is_shutdown()):
            rospy.spin()
        cv2.destroyAllWindows()
    '''

#####################################################################################################

# TODO more idiomatic way to give a node number to each instance
# of each kind of node? shouldn't we just put each set in a 
# namespace like /1/cam, /1/tracker, /2/cam, /2/tracker, etc? 
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    root = Tk.Tk()
    diffviewer = DiffViewer(options.nodenum)
    root.mainloop()
    #diffviewer.Main()
