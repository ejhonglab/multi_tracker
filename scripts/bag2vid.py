#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import os
import traceback
import glob
import copy
import sys
from enum import Enum
from distutils.version import LooseVersion, StrictVersion

import cv2
from sensor_msgs.msg import Image
import rosbag

from multi_tracker.msg import DeltaVid


print('Using open cv: ', cv2.__version__)
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print('Open CV 3')
else:
    OPENCV_VERSION = 2
    print('Open CV 2')

class InterpType(Enum):
    NONE = 0
    NEAREST = 1
    LINEAR = 2

# TODO write some round trip tests with a lossless codec / approx with (potentially) lossy codec to be used

class Converter:
    def __init__(self, directory, mode='mono', args=dict()):
        self.directory = os.path.abspath(os.path.expanduser(directory))

        bag_files = glob.glob(os.path.join(self.directory, '*.bag'))

        if len(bag_files) == 0:
            raise IOError('no bagfiles in directory {}'.format(self.directory))

        elif len(bag_files) > 1:
            raise IOError('more than one bagfile in directory {}'.format(self.directory))

        self.bag_file = bag_files[0]
        self.delta_video_topic, freq, self.message_count = self.topic_info()

        self.args = args

        if 'interpolation' in args:
            if args['interpolation'] is None:
                self.interpolation = InterpType.NONE

            elif args['interpolation'] is 'nearest':
                self.interpolation = InterpType.NEAREST
                raise NotImplementedError

            elif args['interpolation'] is 'linear':
                self.interpolation = InterpType.LINEAR
                raise NotImplementedError

        else:
            self.interpolation = InterpType.NONE
        
        # TODO this comparison right?
        if self.interpolation == InterpType.NONE:
            self.desired_frame_rate = freq

        else:
            # TODO get from arg
            self.desired_frame_rate = 30

        self.desired_frame_interval = 1.0 / self.desired_frame_rate

        if 'timestamps' in args and args['timestamps']:
            self.write_timestamps = True
        else:
            self.write_timestamps = False

        # TODO queue of some sort?
        self.frames = []
        self.frame_times = []

        self.background_image = None
        self.background_img_filename = None
        self.mode = mode

        self.videowriter = None
        self.video_filename = self.bag_file[:-4] + '.avi'

        if os.path.exists(self.video_filename):
            print(self.video_filename, 'already exists. exiting.')
            sys.exit()

        try:
            from tqdm import tqdm
            self.tqdm = tqdm
            self.use_tqdm = True

        except ImportError:
            self.use_tqdm = False
            pass


    def topic_info(self):
        bag = rosbag.Bag(self.bag_file)
        ti = bag.get_type_and_topic_info()
        topics = []
        for t, info in ti.topics.items():
            if info.msg_type == 'multi_tracker/DeltaVid':
                topics.append(t)
        bag.close()
        
        if len(topics) == 0:
            raise ValueError('no topics of type multi_tracker/DeltaVid in bag file.')

        elif len(topics) > 1:
            raise ValueError('bag has multiple topics of type multi_tracker/DeltaVid.')

        topic = topics[0]
        freq = ti.topics[topic].frequency
        count = ti.topics[topic].message_count
        return topic, freq, count


    def load_background_image(self, png_filename):
        """
        Sets self.background_img_filename and attempts to set self.background_image.
        Also slightly reformats ("for ROS hydro"), but this may need fixing.
        """
        self.background_img_filename = png_filename
        basename = os.path.basename(self.background_img_filename)
        full_png_filename = os.path.join(self.directory, basename)
        
        if not os.path.exists(full_png_filename):
            raise IOError('background image file ' + full_png_filename + ' did not exist')

        if not os.path.getsize(full_png_filename) > 0:
            raise IOError('background image file ' + full_png_filename + ' was empty')
        
        self.background_image = cv2.imread(full_png_filename, cv2.CV_8UC1)
        # TODO seems to imply not color, but other code doesnt assert that
        # (fix in delta_video_player.py)
        try:
            self.background_image = self.background_image.reshape([self.background_image.shape[0], self.background_image[1], 1]) # for hydro

        # TODO handle cases by version explicitly or at least specify expected error
        except:
            pass # for indigo
            

    def write_frame(self, image):
        """
        Write a frame to the output file.
        Potentially derived from =/= 1 input frames, as resampled in time.
        """
        if self.mode == 'color':
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        # TODO make this in __init__?
        if self.videowriter is not None:
            self.videowriter.write(image)
        else:
            # TODO detect fourcc to use from input extension?
            #fourcc = cv2.cv.CV_FOURCC(*'MP4V')
            format_code = 'XVID'
            if OPENCV_VERSION == 2:
                fourcc = cv2.cv.CV_FOURCC(*format_code)
                self.videowriter = cv2.VideoWriter(self.video_filename, fourcc, \
                    self.desired_frame_rate, (image.shape[1], image.shape[0]), \
                    1 if self.mode == 'color' else 0)

            elif OPENCV_VERSION == 3:
                fourcc = cv2.VideoWriter_fourcc(*format_code)
                self.videowriter = cv2.VideoWriter(self.video_filename, fourcc, \
                    self.desired_frame_rate, (image.shape[1], image.shape[0]), \
                    1 if self.mode == 'color' else 0)

        
    def add_frame_from_deltavid_msg(self, delta_vid):
        if self.background_image is None:
            self.load_background_image(delta_vid.background_image)
            return None, None

        elif self.background_img_filename != delta_vid.background_image:
            self.load_background_image(delta_vid.background_image)
                
        new_image = copy.copy(self.background_image)
        
        if delta_vid.values is not None and len(delta_vid.values) > 0:
            # TODO check not off by one here?
            try:
                # for hydro
                new_image[delta_vid.xpixels, delta_vid.ypixels, 0] = delta_vid.values

            # TODO look for specific error type or handle differently
            except:
                # for indigo
                new_image[delta_vid.xpixels, delta_vid.ypixels] = delta_vid.values

        secs_from_start = (delta_vid.header.stamp - self.start_time).to_sec()

        # TODO resample for constant framerate. maybe warn if deviate much from it?
        if self.interpolation != InterpType.NONE:
            self.frames.append(new_image)
            self.frame_times.append(secs_from_start)
        
        return new_image, secs_from_start


    def closest_timestamp(self, t):
        return round(t.to_secs() / self.desired_frame_rate) * self.desired_frame_rate


    # TODO how to resample online? (i don't think i can load all of the videos into memory)
    # source too fast -> wait until next goal time and average intervening (weighting maybe)
    def process_bag(self):
        # TODO is there a context manager for this?
        bag = rosbag.Bag(self.bag_file)

        if self.write_timestamps:
            ts_fp = open(os.path.join(self.directory, 'frametimes.txt'), 'w')

        # TODO could maybe use bag.get_start_time() (returns a float Unix epoch time)
        self.start_time = None
        self.current_goal_timestamp = 0
        # TODO TODO are these guaranteed to be sorted? how close to? (sort if not?)
        # make one pass through getting times -> sort -> revisit in sorted order (how?)
        iterate_over = bag.read_messages(topics=[self.delta_video_topic])
        if self.use_tqdm:
            iterate_over = self.tqdm(iterate_over, total=self.message_count)

        for topic, msg, t in iterate_over:

            if self.start_time is None:
                self.start_time = msg.header.stamp

            current_frame, current_frame_time = self.add_frame_from_deltavid_msg(msg)

            if (not current_frame is None) and self.interpolation == InterpType.NONE:
                self.write_frame(current_frame)

            if (not current_frame_time is None) and self.write_timestamps:
                print(current_frame_time, file=ts_fp)

            '''
            if current_frame_time >= self.current_goal_timestamp - self.desired_frame_interval / 2:

                # we skipped past a goal time (source too slow between these two frames)
                # make each intervening time a weighted average of the two surrounding frames?
                # or just take closest? (take multiple interpolation options?)
                # TODO maybe i should just define the goals such that i dont need to add half interval to compare? offset initial?
                if current_frame_time > self.current_goal_timestamp + self.desired_frame_interval / 2:
                    # TODO TODO handle case where we dont have a previous frame
                    previous_frame = self.frames[-2]
                    previous_frame_time = self.frame_times[-2]
                    current_frame_time = self.frame_times[-1]
                    
                    # TODO TODO recursion for this part?
                    tmp_goal = self.current_goal_timestamp
                    self.current_goal_timestamp = self.closest_timestamp(current_frame_time)

                    # TODO TODO handle case where there were > 1 frames already in buffer (average those closer one?)
                    while tmp_goal < self.current_goal_timestamp + self.desired_frame_interval / 2:
                        # TODO or weighted average / linear or not
                        closest_frame = current_frame if abs(previous_frame_time - tmp_goal) > abs(current_frame_time - tmp_goal) else previous_frame
                        self.write_frame(closest_frame)
                        tmp_goal += self.desired_frame_interval

                    self.frames = []
                    self.frame_times = []
                    
                # we didn't skip a goal time.
                # TODO it is possible other frames fall in window that would be closest to this goal time
                # wait until we go into region closest to next goal time, and then average all prior
                
                self.current_goal_timestamp += self.desired_frame_interval
            '''

        bag.close()
        if self.write_timestamps:
            ts_fp.close()

        if self.videowriter is not None:
            self.videowriter.release()
            print('Note: use this command to make a mac / quicktime friendly video: avconv -i test.avi -c:v libx264 -c:a copy outputfile.mp4')


if __name__ == '__main__':
    input_dir = os.getcwd()
    args = {'interpolation': None,
            'timestamps': True}
    conv = Converter(input_dir, args=args)
    conv.process_bag()

