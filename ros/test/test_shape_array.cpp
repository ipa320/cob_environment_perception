#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>


#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/Shape.h>

#include <ros/time.h>
#include <math.h>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#define PI 3.14159265
using namespace std;

float
toDegrees (float radians)
{
  return radians * (180 / PI);
}

float
toRadians (float degrees)
{
  return degrees * (PI / 180);
}

int
main (int argc, char **argv)
{

  ros::init (argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cob_3d_mapping_msgs::ShapeArray> ("shapes_array", 1);

  ROS_INFO(" Publishing shape_array.........:");
  ros::Rate loop_rate (1);
  uint32_t seq = 0;

  while (ros::ok ())
  {
    cob_3d_mapping_msgs::ShapeArray sa;
    sa.header.frame_id = "/map";
    sa.header.stamp = ros::Time::now ();
    sa.header.seq = seq++;

    cob_3d_mapping_msgs::Shape s;
    s.params.resize (4);

    s.header.frame_id = "map";
    s.header.stamp = sa.header.stamp;

    s.type = cob_3d_mapping_msgs::Shape::PLANE;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> pc;
    sensor_msgs::PointCloud2 pc2;
    //pcl::io::loadPCDFile("")
/*
    // Normal estimation*
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTree<pcl::PointXYZ>());
      //tree->setInputCloud (pc);
      //ne.setInputCloud (pc);
      //ne.setSearchMethod (tree);
      //ne.setKSearch (20);
      //ne.compute (*normals);
      // normals should not contain the point normals + surface curvatures
*/

    //first shape
    s.params[0] = 0;
    s.params[1] = 0;
    s.params[2] = 1;
    s.params[3] = 0;

    s.centroid.x = 0.5;
    s.centroid.y = 0.5;
    s.centroid.z = 1.0;

    s.color.r = 1;
    s.color.g = 0;
    s.color.b = 0;
    s.color.a = 1;

    pt.x = 0.0;
    pt.y = 0.0;
    pt.z = 1.0;
    pc.push_back (pt);

    pt.x = 1.0;
    pt.y = 0.0;
    pt.z = 1.0;
    pc.push_back (pt);

    pt.x = 1.0;
    pt.y = 1.0;
    pt.z = 1.0;
    pc.push_back (pt);

    pt.x = 0.0;
    pt.y = 1.0;
    pt.z = 1.0;
    pc.push_back (pt);

    pt.x = 0.0;
    pt.y = 0.0;
    pt.z = 1.0;
    pc.push_back (pt);

    /*
     pt.x = 0.0;
     pt.y = 0.0;
     pt.z = 3.0;
     pc.push_back (pt);

     pt.x = 3.0;
     pt.y = 3.0;
     pt.z = 0.0;
     pc.push_back (pt);

     pt.x = 3.0;
     pt.y = 0.0;
     pt.z = 3.0;
     pc.push_back (pt);
     */
    pcl::toROSMsg (pc, pc2);
    s.points.push_back (pc2);

    s.holes.push_back (false);
    sa.shapes.push_back (s);

     //second shape
     s.params[0] = 0;
     s.params[1] = 0;
     s.params[2] = 1;
     s.params[3] = 0;

     s.centroid.x = 14 / 3;
     s.centroid.y = 14 / 3;
     s.centroid.z = 2;

     s.color.r = 0;
     s.color.g = 1;
     s.color.b = 0;
     s.color.a = 1;

     pc.clear ();
     pc2.data.clear ();
     s.points.clear ();

     pt.x = 2;
     pt.y = 6;
     pt.z = 2;
     pc.push_back (pt);

     pt.x = 6;
     pt.y = 2;
     pt.z = 2;
     pc.push_back (pt);

     pt.x = 6;
     pt.y = 6;
     pt.z = 2;
     pc.push_back (pt);

    /*
     pt.x = 2;
     pt.y = 2;
     pt.z = 2;
     pc.push_back (pt);
     */

     // sensor_msgs::PointCloud2 pc2;
     pcl::toROSMsg (pc, pc2);
     s.points.push_back (pc2);
     s.holes.push_back (false);
     sa.shapes.push_back (s);

     //third shape
     s.centroid.x = -16 / 3;
     s.centroid.y = -11 / 3;
     s.centroid.z = -2;

     s.params[0] = 0;
     s.params[1] = 0;
     s.params[2] = -1;
     s.params[3] = 0;

     s.color.r = 0;
     s.color.g = 0;
     s.color.b = 1;
     s.color.a = 1;

     pc.clear ();
     pc2.data.clear ();
     s.points.clear ();

     pt.x = -5;
     pt.y = -5;
     pt.z = -2;
     pc.push_back (pt);

     pt.x = -5;
     pt.y = -1;
     pt.z = -2;
     pc.push_back (pt);

     pt.x = -6;
     pt.y = -5;
     pt.z = -2;
     pc.push_back (pt);


     //sensor_msgs::PointCloud2 pc2;
     pcl::toROSMsg (pc, pc2);
     s.points.push_back (pc2);

     s.holes.push_back (false);
     sa.shapes.push_back (s);

     //fourth shape
     s.centroid.x = -2 / 3;
     s.centroid.y = -2 / 3;
     s.centroid.z = 0;

     s.params[0] = 0;
     s.params[1] = 0;
     s.params[2] = 1;
     s.params[3] = 0;

     s.color.r = 1;
     s.color.g = 0;
     s.color.b = 1;
     s.color.a = 1;

     pc.clear ();
     pc2.data.clear ();
     s.points.clear ();

     pt.x = -1;
     pt.y = 0;
     pt.z = 0;
     pc.push_back (pt);

     pt.x = -1;
     pt.y = -1;
     pt.z = 0;
     pc.push_back (pt);

     pt.x = 0;
     pt.y = -1;
     pt.z = 0;
     pc.push_back (pt);
/*
     pt.x = 0;
     pt.y = -2;
     pt.z = 0;
     pc.push_back (pt);
  */
     //sensor_msgs::PointCloud2 pc2;
     pcl::toROSMsg (pc, pc2);
     s.points.push_back (pc2);

     s.holes.push_back (false);
     sa.shapes.push_back (s);

     //fifth shape
     s.params[0] = 0;
     s.params[1] = 0;
     s.params[2] = -1;
     s.params[3] = 0;

     s.centroid.x = -2;
     s.centroid.y = -1;
     s.centroid.z = -3;

     s.color.r = 1;
     s.color.g = 1;
     s.color.b = 0;
     s.color.a = 1;

     pc.clear ();
     pc2.data.clear ();
     s.points.clear ();

     pt.x = 0.0;
     pt.y = 0.0;
     pt.z = -3.0;
     pc.push_back (pt);

     pt.x = -3.0;
     pt.y = 0.0;
     pt.z = -3.0;
     pc.push_back (pt);

     pt.x = -3.0;
     pt.y = -3.0;
     pt.z = -3.0;
     pc.push_back (pt);
/*
     pt.x = 0.0;
     pt.y = -3.0;
     pt.z = -3.0;
     pc.push_back (pt);
*/
     pcl::toROSMsg (pc, pc2);
     s.points.push_back (pc2);

     s.holes.push_back (false);
     sa.shapes.push_back (s);

    pub.publish (sa);

    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}

