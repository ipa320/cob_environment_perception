#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_3d_mapping_point_map')
import rospy
import actionlib
import sys

from cob_3d_mapping_msgs.msg import *

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print "Please provide the input argument \"start\" or \"stop.\""
        sys.exit()
    rospy.init_node('trigger_mapping')
    client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)
    client.wait_for_server()

    goal = TriggerMappingGoal()
    if sys.argv[1] == "start":
        goal.start = True
    elif sys.argv[1] == "stop":
        goal.start = False
    else:
        print "Input argument has to be \"start\" or \"stop\""
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

