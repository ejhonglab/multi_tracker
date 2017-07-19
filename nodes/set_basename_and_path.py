#!/usr/bin/env python
import rospy
import time

# TODO get rid of this?

if __name__ == '__main__':
    # TODO fix nodenum
    if rospy.get_param('multi_tracker/retracking_original_timestamp', False):
        experiment_basename = time.strftime("%Y%m%d_%H%M%S_N1", time.localtime(rospy.Time.now()))
    else:
        experiment_basename = time.strftime("%Y%m%d_%H%M%S_N1", time.localtime())
    rospy.set_param('multi_tracker/experiment_basename', experiment_basename)
