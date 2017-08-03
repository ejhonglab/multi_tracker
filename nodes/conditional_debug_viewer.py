#!/usr/bin/env python

import rospy
import rospkg
import roslaunch

if __name__ == "__main__":
    # TODO do i need to register the node or just put it in the manifest or something?
    # advantage? private ns?
    rospy.init_node('conditional_debug_viewer')

    launch_file = rospkg.get_path('multi_tracker') + '/launch/conditional_debug_viewer.launch'
    debug_tracker = rospy.get_param('multi_tracker/tracker/debug', False)
    if debug_tracker:
        # TODO this doesnt make a second master or anything does it?
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        launch.start()
