#!/usr/bin/env python

import time

import rospy


if __name__ == '__main__':
    rospy.init_node('set_exp_basename', log_level=rospy.INFO)

    if (rospy.get_param('/use_sim_time', False) and
        rospy.get_param('multi_tracker/retracking_original_timestamp', False)):

        experiment_basename = rospy.get_param('original_basename', None)
        if experiment_basename is None:
            # TODO get rid of linebreaks, so you can see full error
            raise ValueError('need original_basename parameter to be set ' +
                'if using original timestamp. possibly incorrect argument to ' +
                'set_original_basename.py or you are not calling this node?')
    
    else:
        # TODO do i want the node number? i'm kind of inclined to put them all
        # in same place else make N# work as other code
        experiment_basename = time.strftime('%Y%m%d_%H%M%S', time.localtime())

        # TODO might want to also support multiple trackers in one of those
        # namespaces... see how my roi_finder did it again?
        # check this doesn't break stuff

        # Assumes ns always starts with a forward slash.
        ns_parts = rospy.get_namespace().split('/')
        if len(ns_parts) >= 3:
            # For example: '/0/' -> ['','0',''] -> '0'
            experiment_basename += '_' + ns_parts[-2]
    
    rospy.set_param('multi_tracker/experiment_basename', experiment_basename)
