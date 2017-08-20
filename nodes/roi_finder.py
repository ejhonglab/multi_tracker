#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
from sensor_msgs.msg import Image
import cv2
import os
# import dynamic_reconfigure.server

"""
params:
-types of rois to look for (approx size too?)
-thresholding (?) (alternatives?)
-dilation beyond detected ROI
-x,y offsets from detected ROI
-throwaway_n_frames
"""

# TODO hopefully shutting this node wont kill stuff launched with it?
# TODO break out ROI definitions from core tracking launch file, and make another tracking
# launch file that includes the core + ROI defs, the core which will be called here separately
# TODO dynamic reconfigure and display ROIs that will be selected with button to lock them in
# maybe a gui to manually edit / define ROIs too?

class RoiFinder:
    def __init__(self):
        # TODO maybe have this launch file do something that won't be changed
        # (launch core tracking nodes without setting parameters?)
        # so I can keep it in a central location?
        # TODO idiomatic ROS way to get package path? use python script location + relative path?
        THIS_PACKAGE = 'multi_tracker'
        # TODO shorter call for this package path?
        # TODO TODO still valid once installed / using that path?
        # TODO TODO test parameters are still accessible / valid across ROIs?
        self.tracking_launch_file = rospy.get_param('roi_finder/tracking_launch_file', \
                rospkg.RosPack().get_path(THIS_PACKAGE) + '/launch/single_tracking_pipeline.launch')
        #        os.path.join(os.environ['HOME'], \
        #        'catkin/src/multi_tracker/single_tracking_pipeline.launch'))

        # TODO
        #self.namespace = 
        node_namespace = 'roi_finder/'
        self.roi_type = rospy.get_param(node_namespace + 'roi_type', 'rectangles')

        self.valid_roi_types = {'rectangles', 'circles', 'masks'}
        if not self.roi_type in self.valid_roi_types:
            raise ValueError('invalid roi_type. valid types are ' + str(valid_roi_types))

        self.toss_first_n_frames = rospy.get_param(node_namespace + 'toss_first_n_frames', 0)
        self.frames_tossed = 0

        # TODO what happens if init_node is called after / before defining subscribers 
        # and publishers and stuff? what all does it do?
        # (examples have subscribers after and pub before)

        # start node
        rospy.init_node('roi_finder')
        size_image = 128 + 1920 * 1080 * 3
        # TODO really need to store in a variable to prevent something from being trashed?
        # is this self function already usable here?
        rospy.Subscriber('camera/image_raw', Image, self.detect_roi_callback, \
                queue_size=10, buff_size=2*size_image)
        # TODO need to be in a loop?
        rospy.spin()
        # spin just until rois detected?


    def launch_tracking_common(self):
        # TODO could maybe rospy.get_namespace() to get prefix for child nodes?
        # TODO how exactly do private ("~" prefix) names work?
        # TODO condense these calls into some helper function?
        # rospy.on_shutdown(self.shutdown) isn't necessary is it?
        # TODO this doesnt make a second master or anything does it?
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [self.tracking_launch_file])
        # TODO TODO make all nodes names unique somehow, assuming they need to be
        # globally unique?
        launch.start()
        # TODO problems with shutting down elsewhere?
        #launch.shutdown()


    def new_tracker_namespaces(self):
        # TODO fix
        this_node_namespace = rospy.get_namespace()
        # remove prefix first?
        nmax = max([int(ns.split('/')[0]) fo ns in rostopic.list(this_node_namespace)])
        # TODO anything to do to make the namespace? maybe only possible when making node?
        return this_node_namespace + '/' + str(nmax + 1) + '/'


    def set_param(self, param, value):
        rospy.set_param(self.namespace + param, value)


    def launch_a_tracking_pipeline_rectangles(self, left, right, top, bottom):
        # TODO would ideally make a new namespace for each tracker, and populate that
        # with the correct ROI parameters
        prefix = self.new_tracker_namespace()
        self.set_param(prefix + 'multi_tracker/roi_b', bottom)
        self.set_param(prefix + 'multi_tracker/roi_t', top)
        self.set_param(prefix + 'multi_tracker/roi_l', left)
        self.set_param(prefix + 'multi_tracker/roi_r', right)
        self.launch_tracking_common()
        

    def launch_a_tracking_pipeline_circles(self, x, y, radius):
        raise NotImplementedError


    def launch_a_tracking_pipeline_masks(self, mask):
        raise NotImplementedError


    def get_edges(self, frame):
        # convert image from ROS message format
        # TODO use cv_bridge? and in places floris converts?

        # threshold (?) + search for rectangles
        # maybe options for other ROI types
        frame = cv_bridge.convert(frame)

        gray = cvv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # TODO present these args are ROS params
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(gray, 100, 200)
        return edges


    def detect_rectangles(self, frame):
        edges = self.get_edges(frame)
        # TODO
        rois = cv2.houghRectangles(edges)
        return rois


    def detect_circles(self, frame):
        raise NotImplementedError
        edges = self.get_edges(frame)
        #circles = cv2.houghCircles(frame, ...)
        #return circles


    def detect_masks(self, frame):
        raise NotImplementedError
        edges = self.get_edges(frame)
        # TODO how to fill exterior?
        # findContours?
        return mask


    def detect_masks(self, frame, expected_mask):
        raise NotImplementedError
        edges = self.get_edges(frame)
        # convert mask if not gray? gray from binary?
        # better way to get edges of binary image?
        mask_edges = cv2.Canny(expected_mask)
        rois = cv2.hough(frame, mask_edges)
        # TODO what i want to return here kinda depends on how i want to process the ROIs later
        return rois


    # TODO what does Ctrax use to detect the ROIs?
    def detect_roi_callback(self, frame):
        """
        Detect ROIs of specified type and launch an instance of tracking pipeline appropriately.
        """
        if self.frames_tossed < self.toss_first_n_frames:
            self.frames_tossed += 1
            return
        
        found_detect = False
        for attr in dir(self):
            if 'detect_' in attr and self.roi_type in attr:
                f = getattr(self.__class__, attr)
                # could also potentially use inspect.is_method(...). different how?
                # purpose of getattr?
                if callable(f):
                    rois = self.f(frame)
                    found_detect = True
        if not found_detect:
            raise ValueError('no detection method found for roi_type "' + self.roi_type + '"')

        found_launch = False
        for attr in dir(self):
            if 'launch_a_tracking_pipeline_' in attr and self.roi_type in attr:
                f = getattr(self.__class__, attr)
                if callable(f):
                    for r in rois:
                        self.f(*r)
                    found_launch = True
        if not found_launch:
            raise ValueError('no launch method found for roi_type "' + self.roi_type + '"')

        '''
        if self.roi_type == 'rectangles':
            rois = self.detect_rectangles(frame)
            for r in rois:
                self.launch_a_tracking_pipeline_rectangles(*r)

        elif self.roi_type == 'circles':
            rois = self.detect_circles(frame)
            for r in rois:
                self.launch_a_tracking_pipeline_circles(*r)

        elif self.roi_type == 'masks':
            rois = self.detect_masks()
            for r in rois:
                self.launch_a_tracking_pipeline_masks(*r)

        else:
            raise ValueError('invalid roi_type. valid types are ' + str(self.valid_roi_types))
        '''


if __name__ == '__main__':
    rf = RoiFinder()
