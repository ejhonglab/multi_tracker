#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
# latest versions of ROS (from source, particularly) should have a method in here for
# getting topic names, so we wouldn't need to use rosgraph
# import rostopic
from rosgraph.masterapi import Master
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
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
        # TODO need to specify if launch is only in source, as before?
        THIS_PACKAGE = 'multi_tracker'
        # TODO shorter call for this package path?
        # TODO still valid once installed / using that path?
        # TODO TODO test parameters are still accessible / valid across ROIs?
        self.tracking_launch_file = rospy.get_param('roi_finder/tracking_launch_file', \
                rospkg.RosPack().get_path(THIS_PACKAGE) + '/launch/single_tracking_pipeline.launch')

        self.current_node_num = 1

        node_namespace = 'roi_finder/'
        self.roi_type = rospy.get_param(node_namespace + 'roi_type', 'rectangles')

        # TODO populate automatically from those with a launch pipeline and
        # a automatic / manual roi selection function depending on current function
        # TODO factor this kind of check into validator node?
        self.valid_roi_types = {'rectangle', 'circle', 'mask', 'polygon'}
        if not self.roi_type in self.valid_roi_types:
            raise ValueError('invalid roi_type: ' + self.roi_type + '. valid types are ' + str(self.valid_roi_types))

        automatic_roi_detection = rospy.get_param(node_namespace + 'automatic_roi_detection', False)
        if not automatic_roi_detection:
            # a place for the click event callback to store points
            self.points = []

        self.toss_first_n_frames = rospy.get_param(node_namespace + 'toss_first_n_frames', 0)
        self.frames_tossed = 0

        self.bridge = CvBridge()

        # TODO what happens if init_node is called after / before defining subscribers 
        # and publishers and stuff? what all does it do?
        # (examples have subscribers after and pub before)

        # start node
        rospy.init_node('roi_finder')

        camera = 'camera/image_raw'
        queue_size = 10
        # TODO determine automatically
        size_image = 128 + 1920 * 1080 * 3
        # TODO should the buff_size not be queue_size * size_image?
        buff_size = 2*size_image

        if automatic_roi_detection:
            rospy.Subscriber(camera, Image, self.detect_roi_callback, \
                    queue_size=queue_size, buff_size=buff_size)

        else:
            rospy.Subscriber(camera, Image, self.manual_roi_callback, \
                    queue_size=queue_size, buff_size=buff_size)

        # spin just until rois detected?
        rospy.spin()


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
        self.current_node_num += 1
        # TODO problems with shutting down elsewhere?
        #launch.shutdown()
        # decrement current_node_num when shuts down / whenever we manually shutdown?

    """
    def get_topics(self):
        # see issue #946 (which has a commit added recently) for rostopic alternative
        # TODO is /rostopic correct? return type?
        try:
            # the rosbridge cost uses Master('/rosbridge')
            publishers, subscribers, services = Master('/rostopic').getSystemState()
            has_node_num = lambda x: 
            # can you not filter a set?
            return filter(lambda x: any(fnmatch.fnmatch(str(x), glob) for glob in topics_glob), list(set([x for x, _ in publishers] + [x for x, _, in subscribers])))
        # TODO which exception type?
        except:
            return []


    def get_topics_in_namespace(self, namespace):
        raise NotImplementedError
    """

    def new_tracker_namespace(self):
        # TODO fix / test this works
        this_node_namespace = rospy.get_namespace()
        rospy.logwarn('rospy.get_namespace()=' + this_node_namespace)
        # remove prefix first?
        #nmax = max([int(ns.split('/')[0]) for ns in rostopic.list(this_node_namespace)])
        # TODO anything to do to make the namespace? maybe only possible when making node?
        #return this_node_namespace + '/' + str(nmax + 1) + '/'
        return this_node_namespace + str(self.current_node_num) + '/'


    def set_param(self, param, value):
        rospy.set_param(param, value)


    def launch_a_tracking_pipeline_polygons(self, points):
        # TODO would ideally make a new namespace for each tracker, and populate that
        # with the correct ROI parameters
        prefix = self.new_tracker_namespace()
        rospy.logwarn('prefix from new_tracker_namespace=' + prefix)
        # TODO if inputs are arbitrary corners, will need to do some min / maxing to
        # use the roi_* parameters as is
        self.set_param(prefix + 'multi_tracker/roi_points', points)
        self.launch_tracking_common()


    # TODO would only work for rectangle oriented to axes... couldn't find rotatedrectangle in python cv2 dir
    def launch_a_tracking_pipeline_rectangles(self, left, right, top, bottom):
        # TODO would ideally make a new namespace for each tracker, and populate that
        # with the correct ROI parameters
        prefix = self.new_tracker_namespace()
        # TODO if inputs are arbitrary corners, will need to do some min / maxing to
        # use the roi_* parameters as is
        self.set_param(prefix + 'multi_tracker/roi_b', bottom)
        self.set_param(prefix + 'multi_tracker/roi_t', top)
        self.set_param(prefix + 'multi_tracker/roi_l', left)
        self.set_param(prefix + 'multi_tracker/roi_r', right)
        self.launch_tracking_common()
        

    def launch_a_tracking_pipeline_circles(self, x, y, radius):
        raise NotImplementedError


    def launch_a_tracking_pipeline_masks(self, mask):
        raise NotImplementedError


    def get_pixel_coords(self, event, x, y, flags, param):
        # TODO are any of the globals in example necessary? drawing?
        # TODO draw a marker too?
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append([x, y])

        # TODO can i poll while this window is up, outside of the callback?
        # (do i need waitKey? if it's blocking, it seems like it will prevent me from polling
        #  if i need it)


    def show_frame_for_selecting(self, frame):
        window_name = 'Manual ROI selection'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.get_pixel_coords)
        cv2.imshow(window_name, frame)
        '''
        # TODO how to turn off callback when done? just destroy window?
        '''

    def manual_polygons(self, frame):
        """
        Prompt the user to click the corners of each rectangle.
        (allow ctrl-z and ctrl-[(shift-z)/y]?)
        """
        self.show_frame_for_selecting(frame)
        # TODO while something...
        polygons = []
        while True:
            # bitwise and to get the last 8 bytes, so that key states are considered the same
            # whether or not things like num-lock are pressed
            # TODO (maybe i want to know some of the other control keys though?)
            # waitKey delays for >= milliseconds equal to the argument
            key = cv2.waitKey(1) & 0xFF
            # 27 is the escape key
            # TODO prompt key to press to exit. ctrl-s? z/y?
            if key == 27:
                break
            
            #if len(self.points) == 4:
            # TODO prompt to press any / specific key to move to next roi
            elif key != -1:
                polygon = []
                # this won't get cleared will it?
                for p in self.points:
                    polygon.append(p)
                # TODO draw?
                polygons.append(polygon)
                self.points = []
        
        if len(self.points) != 0:
            rospy.logwarn('had points in buffer when <Esc> key ended manual selection.')
        
        # TODO how to only destroy one window? those from this node?
        # (don't want to screw with liveviewer or image_view windows...)
        cv2.destroyAllWindows()
        return polygons


    def manual_rectangles(self, frame):
        """
        Prompt the user to click the corners of each rectangle.
        (allow ctrl-z and ctrl-[(shift-z)/y]?)
        """
        raise NotImplementedError
        return rectangles


    def manual_circles(self, frame):
        raise NotImplementedError


    def manual_mask(self, frame):
        raise NotImplementedError


    def get_edges(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # TODO present these args are ROS params
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(gray, 100, 200)
        return edges


    def detect_rectangles(self, frame):
        raise NotImplementedError
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


    def launch_tracking_pipelines(self, rois):
        """
        """
        found_launch = False
        for attr in dir(self):
            if 'launch_a_tracking_pipeline_' in attr and self.roi_type in attr:
                f = getattr(self.__class__, attr)
                if callable(f):
                    for r in rois:
                        f(self, r)
                    found_launch = True
                    break
        if not found_launch:
            raise ValueError('no launch function found for roi_type "' + self.roi_type + '"')


    # can't see how to easily let launch_tracking_pipeline use this too, but would be nice
    def find_and_call_function(self, frame, prefix, description):
        """
        Finds a function in the instance of this class with prefix in it, and calls that function
        with frame as an (the only) argument following self. Description should describe the
        type of function being sought and will be included in an error message if no function
        is found.
        """
        if self.frames_tossed < self.toss_first_n_frames:
            self.frames_tossed += 1
            return

        try:
            # TODO which encoding to use? 8 or something else?
            frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8')

        except CvBridgeError as e:
            # raise?
            rospy.logerr(e)
            return None
        
        found_func = False
        for attr in dir(self):
            if prefix in attr and self.roi_type in attr:
                f = getattr(self.__class__, attr)
                if callable(f):
                    # TODO put behind debug flag?
                    #rospy.loginfo('calling function: ' + str(f))
                    rois = f(self, frame)
                    found_func = True
                    break

        if not found_func:
            raise ValueError('no ' + description + ' function found for roi_type "' + self.roi_type + '"')
        return rois


    def manual_roi_callback(self, frame):
        """
        Manually select ROIs of specified type and launch an instance of tracking pipeline appropriately.
        """
        rois = self.find_and_call_function(frame, 'manual_', 'manual selection')
        self.launch_tracking_pipelines(rois)


    # TODO what does Ctrax use to detect the ROIs?
    def detect_roi_callback(self, frame):
        """
        Detect ROIs of specified type and launch an instance of tracking pipeline appropriately.
        """
        rois = self.find_and_call_funcion(frame, 'detect_', 'roi detection')
        self.launch_tracking_pipelines(rois)


if __name__ == '__main__':
    rf = RoiFinder()
