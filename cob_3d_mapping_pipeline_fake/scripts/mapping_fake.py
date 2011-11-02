#!/usr/bin/python
import roslib
roslib.load_manifest('cob_3d_mapping_pipeline_fake')

import rospy
import actionlib

from cob_3d_mapping_msgs.srv import *
from cob_3d_mapping_msgs.msg import *

class MappingPipelineFake:
	def __init__(self):
		self.running = False
		self.server = actionlib.SimpleActionServer('trigger_mapping', TriggerMappingAction, self.trigger_continuous_mapping, False)
		self.server.start()
	
	def trigger_continuous_mapping(self, goal):
		feedback = TriggerMappingFeedback()
		result = TriggerMappingResult()
		if self.running and goal.start == False:
			feedback.currentStep.data = "stopping"
			self.server.publish_feedback(feedback)
			self.running = False
			rospy.loginfo("Mapping set to stopped")
			result.message.data = "Mapping is now stopped"
		if not self.running and goal.start:
			feedback.currentStep.data = "starting"
			self.server.publish_feedback(feedback)
			self.running = True
			rospy.loginfo("Mapping set to running")
			result.message.data = "Mapping is now running"
		self.server.set_succeeded(result)
	


if __name__ == "__main__":
	rospy.init_node('3d_mapping_pipeline_fake')
	mapping = MappingPipelineFake()
	rospy.loginfo("Fake 3d mapping ready.")
	rospy.spin()

