#!/usr/bin/env python

import os
import traceback
import cv2
from multi_tracker.msg import DeltaVid
from sensor_msgs.msg import Image

from distutils.version import LooseVersion, StrictVersion
print 'Using open cv: ' + cv2.__version__
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print 'Open CV 3'
else:
    OPENCV_VERSION = 2
    print 'Open CV 2'

# TODO write some round trip tests with a lossless codec / approx with (potentially) lossy codec to be used

class Converter:
    def __init__(self, directory, mode='mono', save_to=''):
        self.directory = directory
        self.background_image = None
        self.background_img_filename = None
        self.mode = mode
        
        self.save_to = save_to
        self.videowriter = None

        # TODO detect from settings
        # TODO calculate largest diff beteen any two frames? close to the largest?
        self.desired_frame_rate = 30
        self.desired_frame_interval = 1.0 / self.desired_frame_rate

        is self.save_to is None:
            raise ValueError("must specify both and input and output files")

        # TODO queue of some sort?
        self.frames = []
        self.frame_times = []


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

        if self.videowriter is not None:
            self.videowriter.write(image)
        else:
            if OPENCV_VERSION == 2:
                self.videowriter = cv2.VideoWriter(self.save_to, cv2.cv.CV_FOURCC(*'MP4V'), self.desired_frame_rate, (image.shape[1], image.shape[0]), 1 self.mode == 'color' else 0)

            elif OPENCV_VERSION == 3:
                self.videowriter = cv2.VideoWriter(self.save_to, cv2.VideoWriter_fourcc(*'MP4V'), self.desired_frame_rate, (image.shape[1], image.shape[0]), 1 if self.mode == 'color' else 0)

        
    def add_frame_from_deltavid_msg(self, delta_vid):
        if self.background_image is None:
            self.load_background_image(delta_vid.background_image)
            return

        elif self.background_img_filename != delta_vid.background_image:
            self.load_background_image(delta_vid.background_image)
                
        new_image = copy.copy(self.background_image)
        
        if delta_vid.values is not None and len(delta_vid.value) > 0:
            # TODO check not off by one here?
            try:
                # for hydro
                new_image[delta_vid.xpixels, delta_vid.ypixels, 0] = delta_vid.values

            # TODO look for specific error type or handle differently
            except:
                # for indigo
                new_image[delta_vid.xpixels, delta_vid.ypixels] = delta_vid.values

        # TODO resample for constant framerate. maybe warn if deviate much from it?
        self.frames.append(new_image)
        self.frame_times.append((delta_vid.header.stamp - self.start_time).to_secs())


    def closest_timestamp(self, t):
        return round(t.to_secs() / self.desired_frame_rate) * self.desired_frame_rate


    # TODO how to resample online? (i don't think i can load all of the videos into memory)
    # source too fast -> wait until next goal time and average intervening (weighting maybe)
    def process_bag(self, bag_file):
        bag = rosbag.Bag(bag_file)
        self.start_time = None
        self.current_goal_timestamp = 0
        # TODO include multi_tracker/delta_video as well
        # TODO TODO are these guaranteed to be sorted? how close to? (sort if not)
        # TODO search bag for messages of correct type, and use that topic if only one
        for topic, msg, t in bag.read_messages(topics=['multi_tracker/1/delta_video']):
            # TODO when will t differ from msg.header 
            # TODO TODO which is appropriate timestamp (t or that in msg)?
            print 't=', t
            print 'msg.header.stamp=', msg.header.stamp

            if self.start_time is None:
                self.start_time = msg.header.stamp
             
            self.add_frame_from_deltavid_msg(msg)

            current_frame_time = self.frame_times[-1]
            current_frame = self.frames[-1]
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

            bag.close()
            if self.videowriter is not None:
                self.videowriter.release()
                print "Note: use this command to make a mac / quicktime friendly video: avconv -i test.avi -c:v libx264 -c:a copy outputfile.mp4"


if __name__ == '__main__':
    directory = os.getcwd()
    conv = Converter(directory, save_to=sys.argv[2])
    conv.process_bag(sys.argv[1])

