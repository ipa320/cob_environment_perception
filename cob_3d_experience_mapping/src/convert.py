#!/usr/bin/python

import csv, sys, math
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
vis = open(sys.argv[4], "w")

dbg_si=0
dbg_od=0

def asRadians(degrees):
    return degrees * math.pi / 180

def getXYpos(relativeNullPoint, p):
    """ Calculates X and Y distances in meters.
    """
    return p['longitude'], p['latitude']
    deltaLatitude = p['latitude'] - relativeNullPoint['latitude']
    deltaLongitude = p['longitude'] - relativeNullPoint['longitude']
    latitudeCircumference = 40075160 * math.cos(asRadians(relativeNullPoint['latitude']))
    resultX = deltaLongitude * latitudeCircumference / 360
    resultY = deltaLatitude * 40008000 / 360
    return resultX, resultY

locs=[]
with open(sys.argv[2], 'rb') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
	for row in spamreader:
		ts = int(row[0]) #timestamp
		if len(row)>2:
			lo = (row[4])
			la = (row[5])
			
			locs.append([ts,lo,la])

print "reading file wifi"
with open(sys.argv[1], 'rb') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
	for row in spamreader:
		ts = int(row[0]) #timestamp
		if len(row)>2:			
			level = int(row[2])
			if level<-75: continue
			#ssid = row[1]+str(level/2)
			ssid = row[1]
			if ssid in bssid:
				ssid = bssid[ssid]
			else:
				bssid[ssid] = no
				ssid = no
				no += 1
			wifis.append( [ts, ssid, level] )
			
			best_l = locs[0]
			for l in locs:
				if abs(best_l[0]-ts)>abs(l[0]-ts):
					best_l = l
			vis.write(str(ssid)+";"+best_l[1]+";"+best_l[2]+"\n")
			
		if len(row)>1 and row[1]=="E":
			msg = SensorInfoArray()
			for w in wifis:
				msg.infos.append(SensorInfo(id=w[1]))
			bag.write('/sim_barks/sensor_info', msg, rospy.Time.from_sec(ts/1000.))
			wifis=[]
			dbg_si+=1

print "reading file steps"
null_point=False
with open(sys.argv[2], 'rb') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
	for row in spamreader:
		ts = int(row[0]) #timestamp
		if len(row)>2:
			step = (row[1]=='true')
			ori = float(row[2])
			lo = float(row[4])
			la = float(row[5])
			acc = float(row[6])
			
			if len(steps)>0 and step:
				msg = Odometry()
				msg.header.stamp = rospy.Time.from_sec(ts/1000.)
				msg.twist.twist.linear.x = step_length
				msg.twist.twist.angular.z = ori-steps[len(steps)-1][2]
				
				if not null_point:
					null_point = {'latitude': la, 'longitude': lo}
					print "null_point", null_point
				x,y = getXYpos(null_point, {'latitude': la, 'longitude': lo})
				msg.pose.pose.position.x = x
				msg.pose.pose.position.y = y
				msg.pose.pose.orientation.w = acc
				
				bag.write('/odom', msg, rospy.Time.from_sec(ts/1000.))
				#steps=[]
				dbg_od+=1
			
			if step: steps.append( [ts, step, ori, lo, la, acc] )
		
bag.close()
vis.close()

print "generated ",dbg_si," sensor and ",dbg_od," odometry messages"
print "DONE"
