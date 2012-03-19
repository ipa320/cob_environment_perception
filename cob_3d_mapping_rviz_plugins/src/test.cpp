/*
 * test.cpp
 *
 *  Created on: 16.03.2012
 *      Author: josh
 */

#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/Shape.h>

#include <ros/time.h>

#include <boost/shared_ptr.hpp>


int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cob_3d_mapping_msgs::ShapeArray>("shapes_array",1);

  ros::Rate loop_rate(1);

  while(ros::ok()) {
    cob_3d_mapping_msgs::ShapeArray sa;
    sa.header.frame_id="/map";

    cob_3d_mapping_msgs::Shape s;

    s.params.push_back(0);
    s.params.push_back(0);
    s.params.push_back(1);
    s.params.push_back(0);
    s.centroid.x = 1;
    s.centroid.y = 1;
    s.centroid.z = 1;
    s.header.frame_id="/map";

    s.color.r = 0;
    s.color.g = 1;
    s.color.b = 0;
    s.color.a = 1;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointXYZ pt;

    pt.x=0;
    pt.y=0;
    pt.z=0;
    pc.push_back(pt);

    pt.x=1;
    pt.y=0;
    pt.z=0;
    pc.push_back(pt);

    pt.x=1;
    pt.y=2;
    pt.z=0;
    pc.push_back(pt);

    pt.x=0;
    pt.y=2;
    pt.z=0;
    pc.push_back(pt);

    pt.x=0.5;
    pt.y=1;
    pt.z=0;
    pc.push_back(pt);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc,pc2);
    s.points.push_back(pc2);

    sa.shapes.push_back(s);

    s.params[0]=1;
    s.params[1]=0;
    sa.shapes.push_back(s);

    pub.publish(sa);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

