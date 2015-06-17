#!/usr/bin/python

import csv, sys
import rosbag
from cob_3d_experience_mapping.msg import SensorInfoArray
from nav_msgs.msg import Odometry

print "convert.py wifi.csv steps.csv out.bag"

step_length = 0.6

bssid={}
wifis=[]
steps=[]

no=0

bag = rosbag.Bag(sys.argv[3], 'w')

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
			wifis.clear()
		
		wifis.append( [ts, ssid, level] )

with open(sys.argv[2], 'rb') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
	for row in spamreader:
		ts = int(row[0]) #timestamp
		step = int(row[1])
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
			steps.clear()
		
		if step: steps.append( [ts, step, ori, lo, la, acc] )
		
