#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('cob_3d_mapping_gazebo')

import rospy
import os
import math
import time

from gazebo.srv import *
from simple_script_server import script
#from subprocess import call
import tf
import getopt


class RecordCylinderScript(script):

    def __init__(self,center,radius,num_steps,intervall):
        #assign members
        self.do_spawn =False
        self.do_tf = False
        self.do_verbose= True
        
        self.intervall = intervall
        
        self.num_steps = num_steps
        self.radius = radius
        self.center = center
        self.srv_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.req_set = SetModelStateRequest()
        self.req_set.model_state.model_name = "robot"
        self.req_set.model_state.reference_frame = "map"
        
        #set robot pose to initial pose
        print "[RecordCylinderScript]--> Set robot to initial pose"
        self.set_init_pose()
        
        # output        
        print "[RecordCylinderScript]--> Paramters:"
        print "                          Center = ( %f , % f )" % (self.center[0],self.center[1])
        print "                          Radius = %f " % self.radius
        print "                          Steps  = %i " % (self.num_steps)
        
        
        if do_tf ==True:
            rospy.init_node("record_cyl")
            self.br = tf.TransformBroadcaster()
        
        



    def set_flags(self,do_tf,do_spawn,do_verbose):
        self.do_tf = do_tf
        self.do_spawn = do_spawn
        self.do_verbose = do_verbose
    
    def set_init_pose(self):
        self.req_set.model_state.pose.position.x = -1.5
        self.req_set.model_state.pose.position.y = -1.5
        self.req_set.model_state.pose.position.z = 0
        self.req_set.model_state.pose.orientation.w = 0.923879533
        self.req_set.model_state.pose.orientation.x = 0
        self.req_set.model_state.pose.orientation.y = 0
        self.req_set.model_state.pose.orientation.z = 0            
        self.res_set = self.srv_set_model_state(self.req_set)    
            
            

 
            
    def Run(self):
    
        if self.do_spawn == True:       
            self.Spawn()
            time.sleep(1)    
        
        
        #create circular positions with orientatios
        positions_x=list()
        positions_y=list()
        alpha      =list()
        step = 0
        PI = math.pi
        phi = 0
        
        while step < self.num_steps:
            phi = (step)*(2*PI / self.num_steps)      
            positions_x.append( self.center[0] + self.radius * math.cos(phi) )
            positions_y.append( self.center[1] + self.radius * math.sin(phi) )
            curr_alpha = phi
            if curr_alpha > PI:
                curr_alpha= - (2*PI) + curr_alpha
                alpha.append(curr_alpha)
            elif abs(curr_alpha -2*PI)< 0.01:
                alpha.append(0)
            else:    
                alpha.append( phi)            
            step =step+1
            

        print "[RecordCylinderScript]--> Trajectory calculated"  
        step = 0                
        for a in positions_x:    
        
            #set model_state to current pose on circle        
            self.req_set.model_state.pose.position.x = positions_x[step]
            self.req_set.model_state.pose.position.y = positions_y[step]
            self.req_set.model_state.pose.position.z = 0
            self.req_set.model_state.pose.orientation.w = 0.923879533
            self.req_set.model_state.pose.orientation.x = 0
            self.req_set.model_state.pose.orientation.y = 0
            self.req_set.model_state.pose.orientation.z = alpha[step]            
            self.res_set = self.srv_set_model_state(self.req_set)
            
            

            if self.do_tf == True:
            
                
                # send transformation of current pose 
                self.br.sendTransform((positions_x[step],positions_y[step], 0),
                    tf.transformations.quaternion_from_euler(0, 0, alpha[step]),
                    rospy.Time.now(),"/map","/head_cam3d_link")

          
            #increment step and let sleep for 1 second    
            step = step+1    
            print "[RecordCylinderScript]--> Assumed position % i  of  % i" % (step,self.num_steps)
            
            time.sleep(self.intervall)
            
            
            
    def Spawn(self):
        #spawn cylinder
        if do_verbose == False:
            print "[RecordCylinderScript]--> No Verbose Output"
            os.system("roslaunch cob_3d_mapping_gazebo spawn_cylinder.launch >/dev/null")
        else:
            os.system("roslaunch cob_3d_mapping_gazebo spawn_cylinder.launch ")
        print "[RecordCylinderScript]--> Spawning Cylinder" 
        
        

        
                
        
if __name__ == "__main__":
    #set flags to default values
    do_tf = False
    do_spawn = False
    do_verbose = True
    
    
    intervall = 0.3
    num_steps = 12
    radius = 1
    center = (0,0)
    # parse command line options
    try:
        opts, args = getopt.getopt(sys.argv[1:], "h", ["help"])
    except getopt.error, msg:
        print msg
        print "for help use --help"
        sys.exit(2)
    # process options
    for o, a in opts:
        if o in ("-h", "--help"):
            print "\n\nUsage:\t rosrun RecordCylinder.py [optional arguments]"
            print"arguments:\t v .............. disable verbose output of spawned cylinder"
            print"          \t spawn .......... spawn cylinder"
            print"          \t tf ............. publish transform in frame map"
            print"          \t I [time]........ set rest intervall at every step to time"
            print"          \t N [#] .......... number of steps on trajectory"
            print"          \t C [x0][y0][r] .. set parameters of trajectory circle"
            print"-------------------------------------------------------------------------"
            print"default values\t V = True , spawn = False , tf = False , I 0.3 , C 0 0 1 , N 12 "   
            sys.exit(0)

            
    # process arguments
    for i in range(len(args)):

        arg = args[i]
        if (arg in 'V')  == True:
            print "no verbose activated"
            do_verbose = False
            continue        
        elif (arg in 'spawn')  == True:
            print "spawn = true"
            do_spawn = True            
            continue
        elif (arg in "tf")  == True:
            print "tf = true"
            do_tf = True
            continue
        elif(arg in "I") == True:
            print ("intervall set to %s" % args[i+1])
            intervall = (float)(args[i+1])
            continue
        elif(arg in "N") == True:
            print ("number of steps set to %s" % args[i+1] )
            num_steps = (float)(args[i+1])         
            continue              
        elif(arg in "C") == True:
            print ("circle paramaters set")
            center[0] = (float)(args[i+1])
            center[1] = (float)(args[i+2])
            radius = (float)(args[i+3])            
            continue
                    

             
            
                
    #parameters for trajectory


    
    #initialize script
    SCRIPT = RecordCylinderScript(center,radius,num_steps,intervall)
    SCRIPT.set_flags(do_tf,do_spawn,do_verbose)
    
    #run script
    SCRIPT.Run()

        



