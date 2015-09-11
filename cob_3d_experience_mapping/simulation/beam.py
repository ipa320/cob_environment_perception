#!/usr/bin/python

portals=[]
robot="/robot0"

#poses
#1:  4, 27
#2:  10, 27
#3:  16, 27
#4:  4, 19
#5:  10, 19
#6:  16, 19, True

#7:  4, 12.3
#8:  10, 12.3
#9:  16, 12.3
#10: 4, 3
#11: 10, 3
#12: 16, 4, True

#13: 29, 27
#14: 28, 8, True
#15: 54, 26
#16: 41, 23
#17: 41, 7
#18: 32, 14.5
#19: 49, 14.5
room1=[ [4, 27], [10, 27], [16, 27],  [4, 19], [10, 19], [16, 19, True]]
room2=[ [4, 12.3], [10, 12.3], [16, 12.3],  [4, 3], [10, 3], [16, 4, True]]
floor=[ [29, 27], [28, 8, True], [54, 26],  [41, 23], [41, 7], [32, 14.5], [49, 14.5]]

poses=[room1,room2,floor] # own room


#default map
room1=[ [1.9,1.9, True], [6.7,1.4], [4.7, 4.7] ]
room2=[ [13.2,1.65, True] ]
room3=[ [13.75,12.1, True], [14,8] ]
room4=[ [9.4,8.9], [2.4,9.7], [1.8,13.5, True], [5.8,12.4], [9.9,11.75] ]

poses=[room1,room2,room3,room4] # own room



visited=[]
for p in poses:
	visited.append([0]*len(p))	


def add_portal(x,y,r, aim_x, aim_y, name=""):
	global portals
	portals.append( [x,y,r*r, aim_x,aim_y, name] )

import rospy, math, random
from nav_msgs.msg import Odometry
from stdr_msgs.srv import MoveRobot
from stdr_msgs.srv import MoveRobotRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult

pub_moveto=None
pub_pose=None

def beam(x,y,theta=0):
	global robot
	global pub_pose
	
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
	  return

	pose = PoseWithCovarianceStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "world"
	
	pose.pose.pose.position.x = x
	pose.pose.pose.position.y = y
	pose.pose.pose.orientation.w = 1
	
	var_trans = 0.1
	var_rot = 0.1
	pose.pose.covariance = \
[var_trans, 0, 0, 0, 0, 0, 
0, var_trans, 0 ,0 ,0 ,0,
0, 0, 0, 0, 0 ,0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, var_rot]
	
	rospy.sleep(0.25)
	pub_pose.publish(pose)
	rospy.sleep(0.1)

def goto(pos):
	global pub_moveto
	
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map_static"
	
	pose.pose.position.x = pos[0]
	pose.pose.position.y = pos[1]
	pose.pose.orientation.w = 1
	
	pub_moveto.publish(pose)
	
def callback(data):
	global portals
	for p in portals:
		dist2 = pow(p[0]-data.pose.pose.position.x,2)+pow(p[1]-data.pose.pose.position.y,2)
		if dist2<=p[2]:
			rospy.loginfo("beaming: "+p[5])
			beam(p[3],p[4])
			return
	
aim = poses[1][0]
last_floor = poses[1]
def callback2(data):
	global poses
	global aim
	global last_floor
	radius2 = 3
	
	#dist2 = pow(aim[0]-data.pose.pose.position.x,2)+pow(aim[1]-data.pose.pose.position.y,2)
	#print math.sqrt(dist2)
	if True: #dist2<=radius2:
		if len(aim)>=3:
			print "beaming"
			rospy.loginfo("beaming")
			aim=None
			beamto=None
			while aim==None:
				f = random.choice(poses)
				if f==last_floor: continue
				for p in f:
					if len(p)>=3 and p!=aim:
						beamto=p
				p = random.choice(f)
				if p!=beamto:
					aim=p
					last_floor = f
						
			beam(beamto[0],beamto[1])
			goto(aim)
			print "beaming done"
		else:
			p = aim
			while p==aim:
				p = random.choice(last_floor)
			aim=p
			goto(aim)
			
def listener():
	global robot
	global pub_moveto
	global pub_pose
	
	add_portal(0,0,5, 10,10, "zero")
	
	rospy.init_node('sim_beam')
	rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback2)
	pub_moveto = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=2)
	pub_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=2)
	# spin() simply keeps python from exiting until this node is stopped
	
	rospy.sleep(0.5)
	goto(aim)
	
	rospy.spin()

listener()
