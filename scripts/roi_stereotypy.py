#!/usr/bin/env python

"""
Displays ROIs and their labels to see whether ROIs are consistent enough to be
saved / loaded without modification.
"""

from __future__ import print_function
from __future__ import division

import os
import glob
import re

import numpy as np
import cv2
import rosparam
import matplotlib.pyplot as plt


# Just copied from roi_finder for convenience...
def find_roi_namespace(key, params):
    if type(params) is list:
	for ps, ns in params:
	    if ns == key:
		return params
	    else:
		ret = find_roi_namespace(key, ps)
		if not ret is None:
		    return ret
	return None

    elif type(params) is dict:
	if key in params:
	    return params[key]

	else:
	    for v in params.values():
		ret = find_roi_namespace(key, v)
		if not ret is None:
		    return ret
	    return None


times_and_rois = []
# Change as necessary. I got this from typing "file full_background.png", using
# one of the full_background.pngs output from my experiments.
# (I had to transpose from 640x480)
frame_size = (480, 640, 3)

for d in glob.glob(os.environ['DATA_DIR'] + '/*/'):
    files = glob.glob(d + '*/compressor_rois_*.yaml')
    if len(files) == 0:
        continue

    for f in files:
        paramlist = rosparam.load_file(f)
        ns_param_dict = find_roi_namespace('delta_compressor', paramlist)

        rois = dict()
        for k, v in ns_param_dict.items():
            try:
                n = int(k)
            except:
                continue
            rois[n] = v['roi_points']

        acquisition_time = re.search('\d{8}_\d{6}', f).group(0)
        times_and_rois.append((acquisition_time, rois))

# TODO get range of times ROIs were generated on, and interpolate color along
# range? (or just constant increments, in order?)

start = '20180702'
n_past_start = sum([x[0] >= start for x in times_and_rois])

cmap = plt.cm.viridis
color_increment = 1.0 / n_past_start
def c(scalar):
    r, g, b, _ = cmap(scalar)
    return b, g, r

all_roi_nums = set()
roi_num_counts = dict()
for _, rois in times_and_rois:
    for n in rois.keys():
        all_roi_nums.add(n)
        if n in roi_num_counts:
            roi_num_counts[n] = roi_num_counts[n] + 1
        else:
            roi_num_counts[n] = 1

print('Frequency of each ROI number:')
for n, count in roi_num_counts.items():
    print('{} - {}'.format(n, count))

for curr_n in all_roi_nums | {None}:
    img = np.ones(frame_size)
    curr_color_scalar = 0.0
    for time, rois in sorted(times_and_rois, key=lambda x: x[0]):
        if time < start:
            continue

        for n, r in rois.items():
            if (not curr_n is None) and curr_n != n:
                continue

            # TODO want the outline of each convex hull
            hull = cv2.convexHull(np.array(r))
            cv2.drawContours(img, [hull], -1, c(curr_color_scalar))

        # TODO draw number as well
        curr_color_scalar += color_increment
        # TODO draw scale bar indicating which colors are earlier / later

    if curr_n is None:
        name = 'roi_stereotypy'
    else:
        name = 'chamber {}'.format(curr_n)

    cv2.namedWindow(name)
    # To make sure window is not off the edge of the screen.
    cv2.moveWindow(name, 100, 100)
    cv2.imshow(name, img)

cv2.waitKey(0)
# TODO also exit whole script if 'x' on window is pressed
cv2.destroyAllWindows()
