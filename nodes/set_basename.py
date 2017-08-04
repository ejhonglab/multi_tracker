#!/usr/bin/env python
import rospy
import time

# TODO get rid of this?

if __name__ == '__main__':
    # TODO fix nodenum
    rospy.init_node('set_exp_basename', log_level=rospy.INFO)
    if rospy.get_param('multi_tracker/retracking_original_timestamp', False):
        experiment_basename = rospy.get_param('original_basename', None)
        if experiment_basename is None:
            raise ValueError('need original_basename parameter to be set if using ' + \
                'original timestamp. possibly incorrect argument to set_original_basename.py' + \
                ' or you are not calling this node?')
        
    else:
        # TODO make N# work via detecting the parent namespace
        experiment_basename = time.strftime("%Y%m%d_%H%M%S_N1", time.localtime())
     
    rospy.set_param('multi_tracker/experiment_basename', experiment_basename)
