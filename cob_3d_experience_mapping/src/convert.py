#!/usr/bin/python

import csv, sys
import rosbag, rospy
from cob_3d_experience_mapping.msg import SensorInfoArray, SensorInfo
from nav_msgs.msg import Odometry

print "convert.py wifi.csv steps.csv out.bag"

step_length = 0.6

bssid={}
wifis=[]
steps=[]

no=0

bag = rosbag.Bag(sys.argv[3], 'w')

dbg_si=0
dbg_od=0

print "reading file wifi"
with open(sys.argv[1], 'rb') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
	for row in spamreader:
		ts = int(row[0]) #timestamp
		ssid = row[1]
		level = int(row[2])
		if ssid in bssid:
			ssid = bssid[ssid]
		else:
			bssid[ssid] = no
			ssid = no
			no += 1
			
		if len(wifis)>0 and wifis[0][0]!=ts:
			msg = SensorInfoArray()
			for w in wifis:
				msg.infos.append(SensorInfo(id=w[1]))
			bag.write('/sim_barks/sensor_info', msg, rospy.Time.from_sec(wifis[0][0]/1000.))
			wifis=[]
			dbg_si+=1
		
		wifis.append( [ts, ssid, level] )

print "reading file steps"
with open(sys.argv[2], 'rb') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
	for row in spamreader:
		ts = int(row[0]) #timestamp
		step = (row[1]=='true')
		ori = float(row[2])
		lo = float(row[3])
		la = float(row[4])
		acc = float(row[5])
			
		if len(steps)>0 and step:
			msg = Odometry()
			msg.twist.twist.linear.x = step_length
			msg.twist.twist.angular.z = ori-steps[len(steps)-1][2]
			
			msg.pose.pose.position.x = lo
			msg.pose.pose.position.y = la
			#msg.pose.pose.orientation.w = ori
			
			bag.write('/odom', msg, rospy.Time.from_sec(ts/1000.))
			steps=[]
			dbg_od+=1
		
		if step: steps.append( [ts, step, ori, lo, la, acc] )
		
bag.close()

print "generated ",dbg_si," sensor and ",dbg_od," odometry messages"
print "DONE"
