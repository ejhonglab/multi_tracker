#!/usr/bin/env python
import rospy
import time

# TODO get rid of this?

if __name__ == '__main__':
    rospy.init_node('set_exp_basename', log_level=rospy.INFO)
    if rospy.get_param('multi_tracker/retracking_original_timestamp', False):
        experiment_basename = rospy.get_param('original_basename', None)
        if experiment_basename is None:
            raise ValueError('need original_basename parameter to be set if using ' + \
                'original timestamp. possibly incorrect argument to set_original_basename.py' + \
                ' or you are not calling this node?')
        
    else:
        # TODO do i want the node number? i'm kind of inclined to put them all in same place
        # else make N# work as other code
        experiment_basename = time.strftime('%Y%m%d_%H%M%S', time.localtime())
     
    rospy.set_param('multi_tracker/experiment_basename', experiment_basename)
