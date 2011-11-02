#!/usr/bin/python
import roslib
roslib.load_manifest('cob_3d_mapping_pipeline_fake')

import rospy
import actionlib

from cob_3d_mapping_msgs.srv import *
from cob_3d_mapping_msgs.msg import *


if __name__ == "__main__":
	rospy.init_node('client_mapping_fake')
	client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)
	client.wait_for_server()

	goal = TriggerMappingGoal()
	"""Start mapping"""
	goal.start = True
	# Fill in the goal here
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))
	print "Action result:", client.get_result()
	
	"""Stop mapping"""
	goal.start = False
	client.send_goal(goal)
	result = client.wait_for_result(rospy.Duration.from_sec(5.0))
	print "Action result:", client.get_result()

