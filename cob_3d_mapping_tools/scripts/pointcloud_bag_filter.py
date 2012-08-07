#!/usr/bin/python

import argparse

import roslib; roslib.load_manifest('cob_3d_mapping_tools')
import rospy
import rosbag

parser = argparse.ArgumentParser(description="TestDesc")
parser.add_argument("ifile", metavar="IN")
parser.add_argument("ofile", metavar="OUT")

args = parser.parse_args()
bag_in = rosbag.Bag(args.ifile, 'r')
bag_out = rosbag.Bag(args.ofile, 'w')

stamps_int = [
    [1038,127000000],
    [1038,567000000],
    [1038,800000000],
    [1038,964000000],
    [1040,878000000],
    [1042,38000000],
    [1042,93000000],
    [1056,704000000],
    [1056,754000000],
    [1057,942000000],
    [1058,53000000],
    [1058,116000000],
    [1058,537000000]
    ]

stamps_time = []# = [ roslib.rostime.Time().set(i[0],i[1]) for i in stamps_int ]
for i in stamps_int:
    stamps_time.append(roslib.rostime.Time())
    stamps_time[len(stamps_time)-1].set(i[0],i[1])


i=0
for topic, msg, t in bag_in.read_messages():
    if (topic == "/registration/pc_aligned"):
        stamp = msg.header.stamp
        if (stamp in stamps_time):
            print msg.header.stamp.to_time()
            continue

    bag_out.write(topic, msg, t)

bag_in.close()
bag_out.close()
