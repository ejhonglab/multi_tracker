#!/usr/bin/env python

import os
from subprocess import Popen
import Queue
import glob

import rospy
import roslaunch
# latest versions of ROS (from source, particularly) should have a method in
# here for getting topic names, so we wouldn't need to use rosgraph import
# rostopic
import rospkg
from sensor_msgs.msg import Image
# import dynamic_reconfigure.server
from cv_bridge import CvBridge, CvBridgeError
import cv2

from multi_tracker.msg import Point2D, PolygonalROI, RectangularROI, CircularROI
from multi_tracker.srv import RegisterROIs

# TODO break out ROI definitions from core tracking launch file, and make
# another tracking launch file that includes the core + ROI defs, the core which
# will be called here separately
# TODO dynamic reconfigure and display ROIs that will be selected with button to
# lock them in maybe a gui to manually edit / define ROIs too?

# TODO save a background image here (the one used to select the ROIs) so that it
# is at least possible to recreate full movie (w/ static background) for display
class RoiFinder:
    def __init__(self):
        # TODO what happens if init_node is called after / before defining
        # subscribers and publishers and stuff? what all does it do?
        # (examples have subscribers after and pub before)

        # start node
        rospy.init_node('roi_finder')

        # TODO maybe have this launch file do something that won't be changed
        # (launch core tracking nodes without setting parameters?)
        # so I can keep it in a central location?
        # TODO idiomatic ROS way to get package path? use python script location
        # + relative path?
        # TODO need to specify if launch is only in source, as before?
        THIS_PACKAGE = 'multi_tracker'
        # TODO shorter call for this package path?
        # TODO still valid once installed / using that path?
        # TODO TODO test parameters are still accessible / valid across ROIs?
        self.tracking_launch_file = rospy.get_param(
            'roi_finder/tracking_launch_file',
            rospkg.RosPack().get_path(THIS_PACKAGE) +
            '/launch/single_tracking_pipeline.launch')

        self.current_node_num = 1

        node_namespace = 'roi_finder/'
        self.roi_type = rospy.get_param(node_namespace + 'roi_type',
            'rectangles')

        # will not launch any tracking pipelines if this is True
        # but will still register the rois with the delta video node
        self.video_only = rospy.get_param('~video_only', False)
        # TODO populate automatically from those with a launch pipeline and a
        # automatic / manual roi selection function depending on current
        # function
        # TODO factor this kind of check into validator node?
        self.valid_roi_types = {'rectangle', 'circle', 'mask', 'polygon'}
        if not self.roi_type in self.valid_roi_types:
            raise ValueError('invalid roi_type: {}. valid types are {}'.format(
                self.roi_types, self.valid_roi_types))

        load_rois = rospy.get_param(node_namespace + 'load_rois', False)
        automatic_roi_detection = \
            rospy.get_param(node_namespace + 'automatic_roi_detection', False)

        if not automatic_roi_detection:
            # a place for the click event callback to store points
            self.points = []

        self.toss_first_n_frames = rospy.get_param(node_namespace +
            'toss_first_n_frames', 0)

        self.frames_tossed = 0

        self.bridge = CvBridge()

        self.camera = 'camera/image_raw'
        queue_size = 10
        # TODO determine automatically
        size_image = 128 + 1920 * 1080 * 3
        # TODO should the buff_size not be queue_size * size_image?
        buff_size = 2 * size_image

        self.manual_selection_done = False

        # can't just rospy.spin() here because the main thread
        # is the only one that can call launch files (a sequence
        # of functions beginning with a callback can't start
        # a launch file because it can't register signals)
        self.launch_queue = Queue.Queue()
        self.to_kill = []

        if not load_rois:
            if automatic_roi_detection:
                rospy.Subscriber(self.camera, Image, self.detect_roi_callback, \
                        queue_size=queue_size, buff_size=buff_size)

            else:
                self.manual_sub = rospy.Subscriber(self.camera, Image, 
                    self.manual_roi_callback, queue_size=queue_size,
                    buff_size=buff_size)

        else:
            rospy.logwarn('Ignoring roi_finder/automatic_roi_detection, ' + 
                'because roi_finder/load_rois was True.')
            self.load_rois()

        self.main()


    def launch_tracking_common(self, param_dict):
        extra_params = []
        for k, v in param_dict.items():
            if isinstance(k, str) and isinstance(v, str):
                extra_params.append(k + ':=' + v)
            else:
                raise ValueError(
                    'param_dict must have all keys and values be strings')
        
        params = ['roslaunch', 'multi_tracker', 
            'single_tracking_pipeline.launch', 'dump_roi_params:=True', 
            'viewer:=False', 'num:={}'.format(self.current_node_num), 
            'camera:=' + rospy.resolve_name(self.camera)] + extra_params

        self.current_node_num += 1
        rospy.logwarn(params)
        # TODO consider using floris' technique to kill these gently with pgroup
        p = Popen(params)
        self.to_kill.append(p)
        

    # any support there might have been before for setting arguments via
    # roslaunch api seems to have disappeared... will need to use subprocess for
    # now
    """
    def launch_tracking_common(self):
        # TODO could maybe rospy.get_namespace() to get prefix for child nodes?
        # TODO how exactly do private ("~" prefix) names work?
        # TODO condense these calls into some helper function?
        # rospy.on_shutdown(self.shutdown) isn't necessary is it?
        # TODO this doesnt make a second master or anything does it?
        # TODO maybe set is_child=True if exposed somewhere?
        # see roslaunchrunner api
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,
                                                  [self.tracking_launch_file])
        # TODO TODO make all nodes names unique somehow, assuming they need to
        # be globally unique?
        launch.start()
        self.current_node_num += 1
        # TODO problems with shutting down elsewhere?
        #launch.shutdown()
        # decrement current_node_num when shuts down / whenever we manually
        # shutdown?
        self.to_stop.append(launch)
    """

    """
    def get_topics(self):
        # see issue #946 (which has a commit added recently) for rostopic
        # alternative
        # TODO is /rostopic correct? return type?
        try:
            # the rosbridge cost uses Master('/rosbridge')
            publishers, subscribers, services = \
                Master('/rostopic').getSystemState()

            has_node_num = lambda x: 
            # can you not filter a set?
            return filter(lambda x: any(fnmatch.fnmatch(str(x), glob) 
                for glob in topics_glob), list(set([x for x, _ in publishers] +
                [x for x, _, in subscribers])))

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
        #nmax = max([int(ns.split('/')[0])
        #    for ns in rostopic.list(this_node_namespace)])

        # TODO anything to do to make the namespace? maybe only possible when
        # making node?
        #return this_node_namespace + '/' + str(nmax + 1) + '/'
        return this_node_namespace + str(self.current_node_num) + '/'


    def launch_a_tracking_pipeline_polygons(self, points):
        # TODO test repr here works
        param_dict = {'polygonal_roi': 'True', 'roi_points': repr(points)}
        self.launch_tracking_common(param_dict)


    # TODO would only work for rectangle oriented to axes... couldn't find
    # rotatedrectangle in python cv2 dir
    def launch_a_tracking_pipeline_rectangles(self, left, right, top, bottom):
        # TODO if inputs are arbitrary corners, will need to do some min /
        # maxing to use the roi_* parameters as is (or just cv2 boundingBox /
        # rect)
        param_dict = {'rectangular_roi': 'True', 'roi_b': str(bottom),
            'roi_t': str(top), 'roi_l': str(left), 'roi_r': str(right)}
        self.launch_tracking_common(param_dict)
        

    def launch_a_tracking_pipeline_circles(self, x, y, radius):
        raise NotImplementedError


    def launch_a_tracking_pipeline_masks(self, mask):
        raise NotImplementedError


    def get_pixel_coords(self, event, x, y, flags, param):
        # TODO draw a marker too?
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append([x, y])
            rospy.loginfo('Added point ' + str([x, y]))


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
        rospy.loginfo('Click corners of the polygonal ROI. Press any key to ' +
            'store the points added so far as an ROI. Press <Esc> to close ' +
            'manual selection and launch tracking pipelines.')
        polygons = []
        while True:
            # bitwise and to get the last 8 bytes, so that key states are
            # considered the same whether or not things like num-lock are
            # pressed
            # TODO (maybe i want to know some of the other control keys though?)
            # waitKey delays for >= milliseconds equal to the argument

            # TODO idk why people said it was -1... maybe in 2s
            # complement, but not as far as python seems to care
            # it seems to return 255 when no key pressed
            key = cv2.waitKey(100)
            # adds invariance to things like the state of the num-lock key
            # supposedly
            masked_key = key & 0xFF
            # 27 is the escape key
            # ctrl-s? z/y?
            if masked_key == 27:
                break

            #if len(self.points) == 4:
            # TODO prompt to press any / specific key to move to next roi
            elif masked_key != 255:
                polygon = []
                # this won't get cleared will it?
                for p in self.points:
                    polygon.append(p)
                # TODO draw?
                if len(polygon) < 3:
                    rospy.logerr('key press with less than 3 points in ' +
                        'buffer. need at least 3 points for a polygon. ' +
                        'points still in buffer.')
                else:
                    rospy.loginfo('Added polygon from current points. ' + 
                        'Resetting current points.')

                    polygons.append(polygon)
                    self.points = []
        
        if len(self.points) != 0:
            rospy.logwarn(
                'had points in buffer when <Esc> key ended manual selection.')
        
        # TODO how to only destroy one window? those from this node?
        # (don't want to screw with liveviewer or image_view windows...)
        cv2.destroyAllWindows()
        self.manual_selection_done = True
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
        # TODO what i want to return here kinda depends on how i want to process
        # the ROIs later
        return rois


    def load_polygons(self, params):
        """
        """
        rospy.logwarn('load_polygons with params=' + str(params))
        rois = []
        for k, v in params.items():
            try:
                n = int(k)
            except:
                continue

            if 'roi_points' in v:
                rospy.logwarn('appending roi ' + str(v['roi_points']))
                rois.append(v['roi_points'])
            else:
                rospy.logwarn('numbered namespace without polygonal roi. ' + 
                    'experiment done with different roi type?')
        return rois


    def launch_tracking_pipelines(self, rois):
        """
        """
        found_launch = False
        for attr in dir(self):
            if 'launch_a_tracking_pipeline_' in attr and self.roi_type in attr:
                f = getattr(self.__class__, attr)
                if callable(f):
                    # TODO put behind debug flags
                    #rospy.logwarn('ROIS = ' + str(rois))
                    for r in rois:
                        #rospy.logwarn('THIS ROI = ' + str(r))
                        rospy.logwarn(
                            'starting one tracking pipeline launch file')

                        f(self, r)
                        # TODO remove me? longer?
                        # TODO TODO only remove me when sim_time is set?
                        #rospy.sleep(1)
                    found_launch = True
                    break
        if not found_launch:
            raise ValueError(
                'no launch function found for roi_type "' + self.roi_type + '"')


    # can't see how to easily let launch_tracking_pipeline use this too, but
    # would be nice
    def find_and_call_function(self, prefix, description, frame=None,
            params=None):
        """
        Finds a function in the instance of this class with prefix in it, and
        calls that function with frame as an (the only) argument following self.
        Description should describe the type of function being sought and will
        be included in an error message if no function is found.
        """
        if not frame is None:
            if self.frames_tossed < self.toss_first_n_frames:
                self.frames_tossed += 1
                return

            try:
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
                    if not frame is None:
                        rois = f(self, frame)
                    elif not params is None:
                        rois = f(self, params)
                    else:
                        raise ValueError(
                            'either params or frame needs to be specified')

                    found_func = True
                    break

        if not found_func:
            raise ValueError('no ' + description + 
                ' function found for roi_type "' + self.roi_type + '"')

        return rois


    def load_rois(self):
        """
        """
        # TODO might need load_manifest('rosparam')?
        import rosparam
        # TODO also check in current directory?
        #files = glob.glob('compressor_rois_*.yaml')
        files = glob.glob(os.path.join(rospy.get_param('source_directory'), 
            'compressor_rois_*.yaml'))

        if len(files) < 1:
            rospy.logfatal(
                'Did not find any files matching compressor_rois_*.yaml')
            return []

        elif len(files) > 1:
            rospy.logfatal(
                'Found too many files matching compressor_rois_*.yaml')
            return []

        filename = os.path.abspath(files[0])
        # get the parameters in the namespace of the name we want
        # TODO find roi specifiers wherever they are, in the future
        paramlist = rosparam.load_file(filename)
        ns = 'delta_compressor'
        ns_param_dict = self.find_roi_namespace(ns, paramlist)
        if ns_param_dict is None:
            rospy.logfatal('could not find parameter namespace: ' + ns)
            return

        rois = self.find_and_call_function('load_', 'parameter dump loading',
            params=ns_param_dict)
        rospy.logwarn('loaded rois:' + str(rois))
        self.launch_queue.put(rois)


    # maybe make static
    def find_roi_namespace(self, key, params):
        if type(params) is list:
            for ps, ns in params:
                if ns == key:
                    return params
                else:
                    ret = self.find_roi_namespace(key, ps)
                    if not ret is None:
                        return ret
            return None

        elif type(params) is dict:
            if key in params:
                return params[key]

            else:
                for v in params.values():
                    ret = self.find_roi_namespace(key, v)
                    if not ret is None:
                        return ret
                return None


    def manual_roi_callback(self, frame):
        """
        Manually select ROIs of specified type and launch an instance of
        tracking pipeline appropriately.
        """
        rois = self.find_and_call_function('manual_', 'manual selection', 
            frame=frame)
        self.launch_queue.put(rois)
        #self.launch_tracking_pipelines(rois)


    # TODO what does Ctrax use to detect the ROIs?
    def detect_roi_callback(self, frame):
        """
        Detect ROIs of specified type and launch an instance of tracking
        pipeline appropriately.
        """
        rois = self.find_and_call_funcion('detect_', 'roi detection',
            frame=frame)
        self.launch_queue.put(rois)
        #self.launch_tracking_pipelines(rois)


    def register_rois(self, rois):
        rospy.wait_for_service('register_rois')
        try:
            register = rospy.ServiceProxy('register_rois', RegisterROIs)
            l = []
            if self.roi_type == 'rectangle':
                raise NotImplementedError
                for r in rois:
                    rect = RectangularROI()
                    '''
                    rect.t = 
                    rect.b = 
                    rect.l = 
                    rect.r = 
                    '''
                    l.append(rect)
                register(l, [], [])
                    
            elif self.roi_type == 'circle':
                raise NotImplementedError
                register([], [], l)
            
            elif self.roi_type == 'polygon':
                for r in rois:
                    poly = []
                    for p in r:
                        poly.append(Point2D(p[0], p[1]))
                    l.append(PolygonalROI(poly))
                register([], l, [])
            
            elif self.roi_type == 'mask':
                raise NotImplementedError('mask not supported w/ register_rois')
        except rospy.ServiceException as exc:
            rospy.logfatal('service did not process request: ' + str(exc))


    def main(self):
        """
        Checks for launch requests and executes them.
        """
        rois = None
        while not rospy.is_shutdown():
            if not self.launch_queue.empty():
                rois = self.launch_queue.get()
                if not self.video_only:
                    self.launch_tracking_pipelines(rois)
                # tries to send ROIs (to delta_video node)
                self.register_rois(rois)

            if self.manual_selection_done:
                self.manual_sub.unregister()
                if self.launch_queue.empty() and rois is None:
                    # TODO why is this not being reached?
                    rospy.logerr(
                        'Manual selection closed without selecting any ROIs!')
                    break

        for p in self.to_kill:
            p.kill()


if __name__ == '__main__':
    rf = RoiFinder()
