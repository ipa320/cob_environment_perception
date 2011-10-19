#!/usr/bin/python

PKG = 'cob_env_model'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
#import unittest

from generic_mapping_states import *

class TestStates:
	def __init__(self, *args):
		rospy.init_node('test_states')

	def test_approach_pose(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.pose = "home"

		# open the container
		with SM:
			smach.StateMachine.add('INITIALIZE', initialize(),
				transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)

# main
if __name__ == '__main__':
    test = TestStates()
    test.test_approach_pose()
