#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import os
import glob
import sys
from enum import Enum
from distutils.version import LooseVersion, StrictVersion
import argparse
import warnings

import numpy as np
import cv2
# TODO add this as rosdep or import it behind '-c' flag specification
import pandas as pd
import rosbag

from multi_tracker.msg import DeltaVid


# TODO factor into some multi_tracker util thing if we are going to duplicate
# this portion (+ put all prints behind some (ros?) debug flag / ros logdebug
#print('Using open cv: ', cv2.__version__)
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    #print('Open CV 3')
else:
    OPENCV_VERSION = 2
    #print('Open CV 2')

class InterpType(Enum):
    NONE = 0
    NEAREST = 1
    LINEAR = 2

# TODO write some round trip tests with a lossless codec / approx with
# (potentially) lossy codec to be used

class RunningStats:
    """
    Adapted from Marc Liyanage's answer here:
    https://stackoverflow.com/questions/1174984

    ...which is itself based on:
    https://www.johndcook.com/blog/standard_deviation/

    See also "Welford's online algorithm" from this Wikipedia page:
    https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    """
    def __init__(self, initial_frame):
        self.n = 1

        # NOTE: it seems that without always immediately converting to
        # np.float64 (from uint8 the frames are initially represented as), the
        # minimum variance can sometimes be negative. Not 100% sure this could
        # not happen despite this change, but some discussion of this algorithm
        # I found online indicates this algorithm should not have this problem
        # (a problem that some other algorithms *do* have).
        self.old_m = self.new_m = initial_frame.astype(np.float64)
        self.old_s = np.zeros_like(initial_frame, dtype=np.float64)
        self.new_s = np.zeros_like(initial_frame, dtype=np.float64)

    def push(self, frame):
        frame = frame.astype(np.float64)

        self.n += 1

        self.new_m = self.old_m + (frame - self.old_m) / self.n
        self.new_s = self.old_s + (frame - self.old_m) * (frame - self.new_m)

        self.old_m = self.new_m
        self.old_s = self.new_s

    def mean(self):
        return self.new_m

    def variance(self):
        var = self.new_s / (self.n - 1) if self.n > 1 else self.new_s
        assert var.min() >= 0
        return var

    def standard_deviation(self):
        return np.sqrt(self.variance())


class Converter:
    def __init__(self, directory, mode='mono', **kwargs):
        self.directory = os.path.abspath(os.path.expanduser(directory))

        bag_files = glob.glob(os.path.join(self.directory, '*.bag'))

        if len(bag_files) == 0:
            raise IOError('no bagfiles in directory {}'.format(self.directory))

        elif len(bag_files) > 1:
            raise IOError('more than one bagfile in directory {}'.format(
                self.directory
            ))

        self.bag_file = bag_files[0]

        if 'no_avi' in kwargs and kwargs['no_avi']:
            self.video_filename = None
        else:
            self.video_filename = self.bag_file[:-4] + '.avi'

        if 'timestamps' in kwargs and kwargs['timestamps']:
            self.write_timestamps = True
        else:
            self.write_timestamps = False

        if 'overwrite' in kwargs and kwargs['overwrite']:
            overwrite = True
        else:
            overwrite = False

        if 'overlay_csv' in kwargs and kwargs['overlay_csv']:
            overlay_csv = kwargs['overlay_csv']
            if not os.path.exists(overlay_csv):
                raise IOError('{} (specified via -c) not found'.format(
                    overlay_csv
                ))

            # `self.overlay_df` will be checked by a call to
            # `self._validate_overlay_df()` AFTER `self.start_time` is set in
            # first iteration of the loop in `process_bag`.
            self.overlay_df = pd.read_csv(overlay_csv)
            self.curr_overlay_row = 0
            self._checked_text_overlay = False
        else:
            self.overlay_df = None

        if 'verbose' in kwargs and kwargs['verbose']:
            self.verbose = True
        else:
            self.verbose = False
        self._n_zero_length_delta_vid_values = 0

        if not overwrite and (
            self.video_filename and os.path.exists(self.video_filename)):

            print(self.video_filename, 'already exists.')
            self.video_filename = None

        if self.video_filename is None:
            self.write_timestamps = False

        self.min_projection_fname = None
        if 'min_project' in kwargs and kwargs['min_project']:
            self.min_projection_fname = os.path.join(self.directory,
                '_'.join(os.path.basename(self.bag_file).split('_')[:4]) +
                '_min_projection.png'
            )
            if not overwrite and os.path.exists(self.min_projection_fname):
                print(self.min_projection_fname, 'already exists.')
                self.min_projection_fname = None

        self.std_dev_fname = None
        if 'std_dev' in kwargs and kwargs['std_dev']:
            self.std_dev_fname = os.path.join(self.directory,
                '_'.join(os.path.basename(self.bag_file).split('_')[:4]) +
                '_std_dev.png'
            )
            if not overwrite and os.path.exists(self.std_dev_fname):
                print(self.std_dev_fname, 'already exists.')
                self.std_dev_fname = None

        if (self.video_filename is None and
            self.min_projection_fname is None and self.std_dev_fname is None):
            sys.exit()

        self.delta_video_topic, freq, self.message_count = self.topic_info()

        if self.verbose:
            print('Topic:', self.delta_video_topic)
            print('Average frame rate: {:.2f} Hz'.format(freq))
            print('Number of messages:', self.message_count)

        if 'interpolation' in kwargs:
            if kwargs['interpolation'] is None:
                self.interpolation = InterpType.NONE

            elif kwargs['interpolation'] is 'nearest':
                self.interpolation = InterpType.NEAREST
                raise NotImplementedError

            elif kwargs['interpolation'] is 'linear':
                self.interpolation = InterpType.LINEAR
                raise NotImplementedError
        else:
            self.interpolation = InterpType.NONE
        
        if self.interpolation == InterpType.NONE:
            self.desired_frame_rate = freq
        else:
            # TODO get from arg
            self.desired_frame_rate = 30

        self.desired_frame_interval = 1.0 / self.desired_frame_rate

        # TODO queue of some sort?
        self.frames = []
        self.frame_times = []

        self.background_image = None
        self.background_img_filename = None
        self.mode = mode

        self.videowriter = None
        self.min_frame = None
        self.running_stats = None

        try:
            from tqdm import tqdm
            self.tqdm = tqdm
            self.use_tqdm = True

        except ImportError:
            self.use_tqdm = False
            pass


    def _validate_overlay_df(self):
        """
        Checks some time information in `pd.DataFrame` derived from overlay CSV
        makes sense, including:
        - onsets all before offsets (for a given row, representing one interval)
        - intervals occuring earlier in time are earlier in CSV
        - intervals are non-overlapping
        - no intervals occur before first delta video message in bag file

        Does NOT check intervals against end of bag file video data, as this
        case is handled via warnings in the loop in `process_bag`.
        """
        for x in self.overlay_df.itertuples(index=False):
            # (both of these should be of type `float`)
            assert x.onset < x.offset, 'one onset >= offset'

        # Checks intervals are sorted and non-overlapping.
        for i in range(len(self.overlay_df)):
            if i + 1 >= len(self.overlay_df):
                break

            curr_row = self.overlay_df.iloc[i]
            next_row = self.overlay_df.iloc[i + 1]
            # Not allowing equality so that it's always clear which message
            # should be drawn.
            assert next_row.onset > curr_row.offset

        # Ensuring we are calling this after this is defined, since we need it
        # to check intervals don't precede start of video data.
        assert self.start_time is not None

        # Since we already know the intervals are sorted, we can just check that
        # the first interval (first row) doesn't precede start time.
        start_to_first_onset_s = \
            self.overlay_df.iloc[0].onset - self.start_time.to_sec()

        if self.verbose:
            print('First video frame time to first onset: {:.2f} sec'.format(
                start_to_first_onset_s
            ))

        if start_to_first_onset_s < 0:
            raise ValueError('At LEAST the first onset preceded first video '
                'frame time. This likely reflects a serious error in '
                'coordinating stimulus presentation and video acquisition.'
            )


    def topic_info(self):
        bag = rosbag.Bag(self.bag_file)
        ti = bag.get_type_and_topic_info()
        topics = []
        for t, info in ti.topics.items():
            if info.msg_type == 'multi_tracker/DeltaVid':
                topics.append(t)
        bag.close()
        
        if len(topics) == 0:
            raise ValueError('no topics of type multi_tracker/DeltaVid '
                'in bag file.'
            )

        elif len(topics) > 1:
            raise ValueError('bag has multiple topics of type '
                'multi_tracker/DeltaVid.'
            )

        topic = topics[0]
        freq = ti.topics[topic].frequency
        count = ti.topics[topic].message_count
        return topic, freq, count


    def load_background_image(self, png_filename):
        """
        Sets self.background_img_filename and attempts to set
        self.background_image.

        Also slightly reformats ("for ROS hydro"), but this may need fixing.
        """
        self.background_img_filename = png_filename
        basename = os.path.basename(self.background_img_filename)
        full_png_filename = os.path.join(self.directory, basename)
        
        if not os.path.exists(full_png_filename):
            raise IOError('background image file ' + full_png_filename +
                ' did not exist'
            )

        if not os.path.getsize(full_png_filename) > 0:
            raise IOError('background image file ' + full_png_filename +
                ' was empty'
            )
        
        self.background_image = cv2.imread(full_png_filename, cv2.CV_8UC1)
        # TODO seems to imply not color, but other code doesnt assert that
        # (fix in delta_video_player.py) (might also need to change read
        # arguments to support color, perhaps conditional on some check as to
        # whether it is color or not)
        try:
            # for hydro
            self.background_image = self.background_image.reshape([
                self.background_image.shape[0], self.background_image[1], 1
            ])
            # TODO check this code actually isn't running before deleting

        # TODO handle cases by version explicitly or at least specify expected
        # error
        except:
            # for indigo
            pass
            

    def write_frame(self, image):
        """
        Write a frame to the output file.
        Potentially derived from =/= 1 input frames, as resampled in time.
        """
        if self.mode == 'color':
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        if self.videowriter is None:
            # TODO detect fourcc to use from input extension?
            #fourcc = cv2.cv.CV_FOURCC(*'MP4V')
            # TODO maybe warn that the data is being compressed
            # (it is https://www.xvid.com/faq/) ?
            format_code = 'XVID'
            if OPENCV_VERSION == 2:
                fourcc = cv2.cv.CV_FOURCC(*format_code)
                self.videowriter = cv2.VideoWriter(self.video_filename, fourcc,
                    self.desired_frame_rate, (image.shape[1], image.shape[0]),
                    1 if self.mode == 'color' else 0
                )

            elif OPENCV_VERSION == 3:
                fourcc = cv2.VideoWriter_fourcc(*format_code)
                self.videowriter = cv2.VideoWriter(self.video_filename, fourcc,
                    self.desired_frame_rate, (image.shape[1], image.shape[0]),
                    1 if self.mode == 'color' else 0
                )

        self.videowriter.write(image)

        
    def add_frame_from_deltavid_msg(self, delta_vid):
        secs_from_start = (delta_vid.header.stamp - self.start_time).to_sec()

        if (self.background_image is None or
            self.background_img_filename != delta_vid.background_image):

            self.load_background_image(delta_vid.background_image)

        assert self.background_image is not None
        new_image = self.background_image.copy()

        # TODO delete if inspection of multi_tracker code that generates this
        # seems to indicate this is not possible. left in cause it was a
        # component of an old check, but seemed to be irrelevant
        assert delta_vid.values is not None

        # This seems to happen each time a new background frame is added (if not
        # on the first)? Actually, much more. For example:
        # `.../choice_20210129_022422$ rosrun nagel_laminar bag2vid -v`
        # includes this in its output:
        # ``
        # and `.../choice_20210129_044604$ rosrun nagel_laminar bag2vid -v`
        # includes: ``
        # ...though there it seems neither test experiment had any additional
        # background frames (beyond the first set) taken.
        if len(delta_vid.values) > 0:
            # NOTE: hydro is older than indigo. if i should try to delete one
            # branch, try deleting the hydro one.
            # TODO check not off by one here?
            try:
                # for hydro
                new_image[delta_vid.xpixels, delta_vid.ypixels, 0] = \
                    delta_vid.values

            # TODO look for specific error type or handle differently
            except:
                # for indigo
                new_image[delta_vid.xpixels, delta_vid.ypixels] = \
                    delta_vid.values
        else:
            self._n_zero_length_delta_vid_values += 1

        # TODO resample for constant framerate.
        # maybe warn if deviate much from it?
        if self.interpolation != InterpType.NONE:
            self.frames.append(new_image)
            self.frame_times.append(secs_from_start)

        return new_image, secs_from_start


    def closest_timestamp(self, t):
        return (round(t.to_secs() / self.desired_frame_rate) *
            self.desired_frame_rate
        )


    # TODO how to resample online? (i don't think i can load all of the videos
    # into memory) source too fast -> wait until next goal time and average
    # intervening (weighting maybe)
    def process_bag(self):
        # TODO is there a context manager for this?
        bag = rosbag.Bag(self.bag_file)

        if self.write_timestamps:
            ts_fp = open(os.path.join(self.directory, 'frametimes.txt'), 'w')

        # TODO could maybe use bag.get_start_time() (returns a float Unix epoch
        # time)
        self.start_time = None
        self.current_goal_timestamp = 0
        # TODO are these guaranteed to be sorted? how close to? (sort if not?)
        # make one pass through getting times -> sort -> revisit in sorted order
        # (how?) (or just assert they are in order by keeping track of last and
        # checking...)
        iterate_over = bag.read_messages(topics=[self.delta_video_topic])
        if self.use_tqdm:
            iterate_over = self.tqdm(iterate_over, total=self.message_count)

        # The third value in each element of `iterate_over` is some sort of
        # time.
        # TODO could maybe check against msg.header.stamp? why even have
        # msg.header.stamp in acquisition of these other times are available for
        # free, and are always consistent (if true)?
        for topic, msg, _ in iterate_over:
            if self.start_time is None:
                self.start_time = msg.header.stamp

                if self.overlay_df is not None:
                    self._validate_overlay_df()

            current_frame, current_frame_time = \
                self.add_frame_from_deltavid_msg(msg)

            if self.interpolation == InterpType.NONE:
                if self.video_filename is not None:
                    # TODO if i do fix the interpolation implementation(s)
                    # later, might want to refactor write_frame into a get_frame
                    # type call so that the overlay can be added in a call
                    # between that and the write_frame call
                    if (self.overlay_df is not None and
                        self.curr_overlay_row < len(self.overlay_df)):

                        curr_time_s = msg.header.stamp.to_sec()
                        row = self.overlay_df.iloc[self.curr_overlay_row]

                        if curr_time_s < row.onset:
                            text = None

                        elif row.onset <= curr_time_s <= row.offset:
                            text = row.overlay
                            assert type(text) is str and len(text) > 0

                        elif row.offset < curr_time_s:
                            self.curr_overlay_row += 1

                            if self.curr_overlay_row >= len(self.overlay_df):
                                text = None
                            else:
                                # Just assuming that the frame rate is such that
                                # we won't completely skip past this interval
                                # (so not checking against the end of this new
                                # row this iteration). If that were not true,
                                # would probably want to implment this as
                                # recursion or something, though we'd also have
                                # bigger issues then...  Mostly just checking
                                # here to support the case where the CSV
                                # specifies intervals where one ends the frame
                                # before the next starts.
                                row = self.overlay_df.iloc[
                                    self.curr_overlay_row
                                ]
                                if row.onset <= curr_time_s:
                                    text = row.text
                                else:
                                    text = None
                        else:
                            assert False, \
                                'previous cases should have been complete'

                        if text is not None:
                            font = cv2.FONT_HERSHEY_PLAIN
                            # Only non-int parameter putText seems to take.
                            font_scale = 5.0
                            # White. Would need to convert all frames to some
                            # colorspace to add a colored overlay.
                            color = 255
                            font_thickness = 5

                            # So we can center the text.
                            (text_width, _), _ = cv2.getTextSize(text, font,
                                font_scale, font_thickness
                            )

                            # (0,0) is at the top left.
                            y_margin = 80
                            bottom_left = (
                                (current_frame.shape[1] - text_width) // 2,
                                current_frame.shape[0] - y_margin
                            )

                            if not self._checked_text_overlay:
                                before_overlay = current_frame.copy()

                            cv2.putText(current_frame, text, bottom_left, font,
                                font_scale, color, font_thickness, cv2.LINE_AA
                            )

                            # TODO maybe also overlay counter to next onset?
                            # or just when not text to overlay?

                            if not self._checked_text_overlay:
                                assert not np.array_equal(
                                    before_overlay, current_frame
                                ), 'did not actually draw anything'

                                self._checked_text_overlay = True
        
                    self.write_frame(current_frame)

                if self.min_projection_fname is not None:
                    if self.min_frame is None:
                        self.min_frame = current_frame
                    else:
                        # This is a different fn than `np.min`
                        self.min_frame = np.minimum(
                            current_frame, self.min_frame
                        )

                if self.std_dev_fname is not None:
                    if self.running_stats is None:
                        self.running_stats = RunningStats(current_frame)
                    else:
                        self.running_stats.push(current_frame)

            if self.write_timestamps:
                print(current_frame_time, file=ts_fp)

            '''
            if (current_frame_time >=
                self.current_goal_timestamp - self.desired_frame_interval / 2):

                # we skipped past a goal time (source too slow between these two
                # frames) make each intervening time a weighted average of the
                # two surrounding frames?  or just take closest? (take multiple
                # interpolation options?)
                # TODO maybe i should just define the goals such that i dont
                # need to add half interval to compare? offset initial?
                if (current_frame_time > self.current_goal_timestamp +
                    self.desired_frame_interval / 2):

                    # TODO TODO handle case where we dont have a previous frame
                    previous_frame = self.frames[-2]
                    previous_frame_time = self.frame_times[-2]
                    current_frame_time = self.frame_times[-1]
                    
                    # TODO TODO recursion for this part?
                    tmp_goal = self.current_goal_timestamp
                    self.current_goal_timestamp = \
                        self.closest_timestamp(current_frame_time)

                    # TODO TODO handle case where there were > 1 frames already
                    # in buffer (average those closer one?)
                    while (tmp_goal < self.current_goal_timestamp +
                        self.desired_frame_interval / 2):

                        # TODO or weighted average / linear or not
                        closest_frame = (current_frame if
                            abs(previous_frame_time - tmp_goal) >
                            abs(current_frame_time - tmp_goal)
                            else previous_frame)

                        self.write_frame(closest_frame)
                        tmp_goal += self.desired_frame_interval

                    self.frames = []
                    self.frame_times = []
                    
                # we didn't skip a goal time.
                # TODO it is possible other frames fall in window that would be
                # closest to this goal time

                # wait until we go into region closest to next goal time, and
                # then average all prior
                
                self.current_goal_timestamp += self.desired_frame_interval
            '''

        if self.verbose:
            print('# of frames with empty delta_vid.values:',
                self._n_zero_length_delta_vid_values
            )

        if self.overlay_df is not None:
            last_offset_to_last_frame = \
                msg.header.stamp.to_sec() - self.overlay_df.iloc[-1].offset

            if self.verbose:
                print('Last offset to last frame: {:.2f} sec'.format(
                    last_offset_to_last_frame
                ))

            if last_offset_to_last_frame < 0:
                warnings.warn('At least one offset happened after time of last'
                    ' frame. This is OK if experiment was intentionally stopped'
                    ' early.'
                )

        # TODO probably (also/exclusively) call all these cleanup functions in
        # an atexit call

        bag.close()
        if self.write_timestamps:
            ts_fp.close()

        if self.videowriter is not None:
            self.videowriter.release()
            print('Note: use this command to make a mac / quicktime ' +
                'friendly video: avconv -i test.avi -c:v libx264 -c:a ' +
                'copy outputfile.mp4'
            )

        if self.min_projection_fname:
            assert self.min_frame is not None
            cv2.imwrite(self.min_projection_fname, self.min_frame)

        if self.std_dev_fname:
            assert self.running_stats is not None
            stddev_frame = self.running_stats.standard_deviation()
            # Because I'm not sure how the OpenCV code would react in this case.
            # It'd probably fail anyway...
            assert stddev_frame.max() <= 255, 'stddev > 255 somewhere'
            # NOTE: this works, but rounds from np.float64 to uint8, so if you
            # try loading this PNG, you will only have the rounded values. If
            # you need analysis on the standard deviation image, it would
            # probably be best to export full precision somewhere.
            cv2.imwrite(self.std_dev_fname, stddev_frame)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--min-project', default=False,
        action='store_true', help='Saves the minimum projection of the video to'
        ' a PNG in current directory.'
    )
    parser.add_argument('-s', '--std-dev', default=False,
        action='store_true', help='Saves the standard deviation image of the'
        ' video to a PNG in current directory.'
    )
    parser.add_argument('-a', '--no-avi', default=False,
        action='store_true', help='Does NOT save a .avi movie in the current '
        'directory.'
    )
    parser.add_argument('-o', '--overwrite', default=False, action='store_true',
        help='Overwrite any file that would be generated, yet already exists.'
    )
    # TODO arg for x,y position (floats in [0,1] probably) of overlay?
    # also maybe font size / color of overlay? or just optional extra columns
    # for that information (+ then it could be changed row-by-row)?
    # TODO TODO arg for specification of roi (via gui unless position passed by
    # other args?) that is used to compute a binary signal (via clustering /
    # other automatic thresholding methods) to compute a signal to be compared
    # against these intervals? (e.g. for an LED hardware indicator of odor pulse
    # to be used to check the times recorded in the stimulus metadata are
    # accurate?) maybe rename overlay-csv arg then, and just have overlay as an
    # optional column? should i just output the signal to a CSV or something to
    # be checked by some other program, or check against intervals in here?
    parser.add_argument('-c', '--overlay-csv', default=None, action='store',
        help='CSV with columns onset, offset, and overlay. onset and offset '
            'must contains Unix timestamps in the ROS bag file, and each onset '
            'must precede the corresponding offset. overlay contains text '
            'to be overlayed on the image between each onset and offset.'
    )
    parser.add_argument('-v', '--verbose', default=False, action='store_true')

    kwargs = {
        'interpolation': None,
        'timestamps': True
    }
    kwargs.update(vars(parser.parse_args()))

    input_dir = os.getcwd()
    conv = Converter(input_dir, **kwargs)
    conv.process_bag()
