
#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/Shape.h>

#include <ros/time.h>
#include <math.h>

#include <boost/shared_ptr.hpp>


int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cob_3d_mapping_msgs::ShapeArray>("shapes_array",1);

  ROS_INFO(" Publishing shape_array.........");
  ros::Rate loop_rate(1);
  uint32_t seq = 0;
  while(ros::ok()) {
    cob_3d_mapping_msgs::ShapeArray sa;
    sa.header.frame_id="/map";
    sa.header.stamp = ros::Time::now();
    sa.header.seq = seq++;

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

    s.type = cob_3d_mapping_msgs::Shape::PLANE;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointCloud<pcl::PointXYZ> pc_1;
    pcl::PointCloud<pcl::PointXYZ> pc_2;
    pcl::PointCloud<pcl::PointXYZ> pc_3;
    pcl::PointXYZ pt;

    float f = 0.0;
    for(int i = 0;i < 99;i++)
    {
      float x = 5 * sin(f + i / 10.0f * 2 * M_PI);
      float y = 5 * sin(f + i / 10.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 10.0f * 2 * M_PI);
      pt.x = i/100;
      pt.y = y;
      pt.z = z;
      pc_1.push_back(pt);

      pt.x = x;
      pt.y = i/100;
      pt.z = z;
      pc_2.push_back(pt);

      pt.x = x;
      pt.y = z/100;
      pt.z = i/100;
      pc_3.push_back(pt);

    }



    pt.x=0;
    pt.y=0;
    pt.z=0;
    pc.push_back(pt);

    pt.x=5;
    pt.y=0;
    pt.z=0;
    pc.push_back(pt);

    pt.x=0;
    pt.y=5;
    pt.z=0;
    pc.push_back(pt);

    pt.x=5;
    pt.y=5;
    pt.z=0;
    pc.push_back(pt);

    pt.x=0;
    pt.y=5;
    pt.z=5;
    pc.push_back(pt);

    pt.x=5;
    pt.y=0;
    pt.z=5;
    pc.push_back(pt);

    pt.x=5;
    pt.y=0;
    pt.z=5;
    pc.push_back(pt);

    pt.x=5;
    pt.y=5;
    pt.z=5;
    pc.push_back(pt);

    pt.x=0;
    pt.y=5;
    pt.z=5;
    pc.push_back(pt);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc,pc2);
    s.points.push_back(pc2);
    sa.shapes.push_back(s);

    /*
    s.color.r = 1;
    s.color.g = 0;
    s.color.b = 0;
    s.color.a = 1;
    sensor_msgs::PointCloud2 pc2_1;
    pcl::toROSMsg(pc_1,pc2_1);
    s.points.push_back(pc2_1);
    sa.shapes.pop_back();
    sa.shapes.push_back(s);

    s.color.r = 0;
    s.color.g = 0;
    s.color.b = 1;
    s.color.a = 1;
    sensor_msgs::PointCloud2 pc2_2;
    pcl::toROSMsg(pc_2,pc2_2);
    s.points.push_back(pc2_2);
    sa.shapes.pop_back();
    sa.shapes.push_back(s);

    pub.publish(sa);

    s.color.r = 1;
    s.color.g = 0;
    s.color.b = 1;
    s.color.a = 1;
    sensor_msgs::PointCloud2 pc2_3;
    pcl::toROSMsg(pc_3,pc2_3);
    sa.shapes.pop_back();
    s.points.push_back(pc2_3);

    s.holes.push_back(false);

    sa.shapes.push_back(s);

    s.params[0]=1;
    s.params[1]=0;
    //sa.shapes.push_back(s);
*/
    pub.publish(sa);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

