#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('cob_3d_mapping_gazebo')
roslib.load_manifest('cob_script_server')

import rospy
import os
import math
import time

from gazebo.srv import *
from simple_script_server import *
#from subprocess import call

import getopt


import tf
class broadcaster (script):

    def __init__(self,in_frame,out_frame):
        #rospy.init_node("trafo_")
        rospy.init_node('tf_map_bc')
        self.br = tf.TransformBroadcaster()
        self.in_frame= in_frame
        self.out_frame = out_frame
        self.srv_get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

       # self.ori.append(self.q.x)
       # self.ori.append(self.q.y)
       # self.ori.append(self.q.z)
       # self.ori.append(self.q.w)
       # self.ori.append(self.q[0])
       # self.ori.append(self.q[1])
       # self.ori.append(self.q[2])
       # self.ori.append(self.q[3])
       # print self.ori[2]
    def Run(self):
        req_get = GetModelStateRequest()
        req_get.model_name = "robot"
        state = self.srv_get_model_state(req_get)
        q = [state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w]
        self.br.sendTransform((state.pose.position.x,state.pose.position.y,state.pose.position.z),q,rospy.Time.now(),self.out_frame,self.in_frame)     
            
            
if __name__ == "__main__":
    tf_br = broadcaster("/map","/base_link")
    while not rospy.is_shutdown():
        tf_br.Run()
        rospy.sleep(0.005)
