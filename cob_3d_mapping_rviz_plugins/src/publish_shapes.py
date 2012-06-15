#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_3d_mapping_rviz_plugins')
import rospy
from cob_3d_mapping_msgs.msg import Shape
from cob_3d_mapping_msgs.msg import ShapeArray
def talker():
    pub = rospy.Publisher('shapes_array', ShapeArray)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        sa = ShapeArray()
        s = Shape()
        s.params = [0,0,0,0,1,1,1]
        s.header.frame_id="/map"
        s2 = Shape()
        s2.params = [0,0,0,0,3,1,1]
        s2.header.frame_id="/map"
        sa.shapes.append(s)
        sa.shapes.append(s2)
        pub.publish(sa)
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass