#!/usr/bin/python

portals=[]
robot="/robot0"

def add_portal(x,y,r, aim_x, aim_y, name=""):
	global portals
	portals.append( [x,y,r*r, aim_x,aim_y, name] )

import rospy
from nav_msgs.msg import Odometry
from stdr_msgs.srv import MoveRobot
from stdr_msgs.srv import MoveRobotRequest

def beam(x,y,theta=0):
	global robot
	
	req = MoveRobotRequest()
	req.newPose.x = x
	req.newPose.y = y
	req.newPose.theta = theta
	
	rospy.wait_for_service(robot+'/replace')
	moverob = rospy.ServiceProxy(robot+'/replace', MoveRobot)
	try:
	  resp1 = moverob(req)
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))

def callback(data):
    global portals
    for p in portals:
		dist2 = pow(p[0]-data.pose.pose.position.x,2)+pow(p[1]-data.pose.pose.position.y,2)
		if dist2<=p[2]:
			rospy.loginfo("beaming: "+p[5])
			beam(p[3],p[4])
			return
    
def listener():
    global robot
    
    add_portal(0,0,5, 10,10, "zero")
    
    rospy.init_node('sim_beam')
    rospy.Subscriber(robot+"/odom", Odometry, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()
