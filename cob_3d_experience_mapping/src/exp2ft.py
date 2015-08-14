#!/usr/bin/env python
# license removed for brevity
import rospy
from ratslam_ros.msg import ViewTemplate
from cob_3d_experience_mapping.msg import SensorInfoArray, SensorInfo

def callback(data):
	global pub
	msg = SensorInfoArray()
	msg.infos.append(SensorInfo(data.current_id))
	pub.publish(msg)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/irat_red/LocalView/Template", ViewTemplate, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
pub = rospy.Publisher('/sim_barks/sensor_info', SensorInfoArray, queue_size=10)
listener()
