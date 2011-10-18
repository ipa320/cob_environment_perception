#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('cob_env_model_gazebo')

import rospy
import os

from gazebo.srv import *
from simple_script_server import script

class MyScript(script):
	def Run(self):
		self.sss.move("torso",[[-0.2,0,-0.2]])
		self.sss.move("head","front")
		self.sss.move("tray","down")
		# Move training table on top of gripper
		srv_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		req_set = SetModelStateRequest()
		req_set.model_state.model_name = "cob3"
		req_set.model_state.pose.position.x = -2.5
		req_set.model_state.pose.position.y = 0
		req_set.model_state.pose.position.z = 0
		req_set.model_state.pose.orientation.w = 0
		req_set.model_state.pose.orientation.x = 0
		req_set.model_state.pose.orientation.y = 0
		req_set.model_state.pose.orientation.z = 0
		req_set.model_state.reference_frame = "map"	
		res_set = srv_set_model_state(req_set)
		self.sss.wait_for_input()

		req_set.model_state.pose.position.x = -3
		res_set = srv_set_model_state(req_set)
		self.sss.wait_for_input()
		
		self.sss.move("torso",[[-0.1,0,-0.2]])
		req_set.model_state.pose.position.x = -3.5
		res_set = srv_set_model_state(req_set)
		self.sss.wait_for_input()
		
		req_set.model_state.pose.position.x = -4
		res_set = srv_set_model_state(req_set)
		self.sss.wait_for_input()

		
if __name__ == "__main__":
	SCRIPT = MyScript()
	SCRIPT.Start()
	#if len(sys.argv) < 2:
	#	print '[Train_LoadObject.py] Please specify the name of the urdf model to be loaded'
	#	sys.exit()
	

		



