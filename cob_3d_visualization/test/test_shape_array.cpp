#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/Shape.h>
#include <cob_3d_mapping_common/polygon.h>

#include <ros/time.h>
#include <math.h>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <pcl/common/eigen.h>
#include <eigen_conversions/eigen_msg.h>

#define PI 3.14159265
using namespace std;

void
firstShape (cob_3d_mapping_msgs::Shape& s, Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f x_axis, Eigen::Vector3f color)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pc;
  sensor_msgs::PointCloud2 pc_msg;

  s.id = 0;

  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, origin, t);
  tf::poseEigenToMsg(t.inverse().cast<double>(), s.pose);

  //first shape
  s.params[0] = normal[0];
  s.params[1] = normal[1];
  s.params[2] = normal[2];
  s.params[3] = origin.dot(normal);

  s.color.r = color[0];
  s.color.g = color[1];
  s.color.b = color[2];
  s.color.a = 1;

  float z = 0;

  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = z;
  pc.push_back (pt);

  pt.x = 1.0;
  pt.y = 0.0;
  pt.z = z;
  pc.push_back (pt);

  pt.x = 1.0;
  pt.y = 1.0;
  pt.z = z;
  pc.push_back (pt);

  pt.x = 0.0;
  pt.y = 1.0;
  pt.z = z;
  pc.push_back (pt);

  pt.x = 0.7;
  pt.y = 0.7;
  pt.z = z;
  pc.push_back (pt);

  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(pc, pc2);
  pcl_conversions::fromPCL(pc2, pc_msg);
  //pcl::toROSMsg (pc, pc2);
  s.points.push_back (pc_msg);

  s.holes.push_back(false);
}

void
secondShape (cob_3d_mapping_msgs::Shape& s, Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f x_axis, Eigen::Vector3f color)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pc, pc_h;
  sensor_msgs::PointCloud2 pc_msg, pc_msg_h;

  s.id = 1;

  s.params[0] = normal[0];
  s.params[1] = normal[1];
  s.params[2] = normal[2];
  s.params[3] = origin.dot(normal);

  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, origin, t);
  tf::poseEigenToMsg(t.inverse().cast<double>(), s.pose);

  s.color.r = color[0];
  s.color.g = color[1];
  s.color.b = color[2];
  s.color.a = 1;

  pt.x = 0;
  pt.y = 0;
  pt.z = 0;
  pc.push_back (pt);

  pt.x = 2;
  pt.y = 0;
  pc.push_back (pt);

  pt.x = 2;
  pt.y = 2;
  pc.push_back (pt);

  pt.x = 0;
  pt.y = 2;
  pc.push_back (pt);

  pt.x = 0.5;
  pt.y = 0.5;
  pc_h.push_back(pt);

  pt.x = 0.5;
  pt.y = 1;
  pc_h.push_back(pt);

  pt.x = 1;
  pt.y = 0.5;
  pc_h.push_back(pt);

  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(pc, pc2);
  pcl_conversions::fromPCL(pc2, pc_msg);
  //pcl::toROSMsg (pc, pc2);
  s.points.push_back (pc_msg);

  pcl::PCLPointCloud2 pc2_h;
  pcl::toPCLPointCloud2(pc_h, pc2_h);
  pcl_conversions::fromPCL(pc2_h, pc_msg_h);
  //pcl::toROSMsg (pc, pc2);
  s.points.push_back (pc_msg_h);

  s.holes.push_back(false);
  s.holes.push_back(true);
}

void
thirdShape (cob_3d_mapping_msgs::Shape& s, Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f x_axis, Eigen::Vector3f color)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pc;
  sensor_msgs::PointCloud2 pc_msg;

  s.id = 2;

  s.params[0] = normal[0];
  s.params[1] = normal[1];
  s.params[2] = normal[2];
  s.params[3] = origin.dot(normal);

  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, origin, t);
  tf::poseEigenToMsg(t.inverse().cast<double>(), s.pose);

  s.color.r = color[0];
  s.color.g = color[1];
  s.color.b = color[2];
  s.color.a = 1;

  pt.x = -5;
  pt.y = -5;
  pt.z = 0;
  pc.push_back (pt);

  pt.x = -5;
  pt.y = -1;
  pt.z = 0;
  pc.push_back (pt);

  pt.x = -6;
  pt.y = -5;
  pt.z = 0;
  pc.push_back (pt);

  //Eigen::VectorXf centroid;
  //pcl::computeNDCentroid (pc, centroid);
  //std::cout<<" centroid : "<<centroid<<"--> X: "<<centroid[0]<<"--> Y: "<<centroid[1]<<"--> Z: "<<centroid[2]<<std::endl;

  /*s.centroid.x = centroid[0];
  s.centroid.y = centroid[1];
  s.centroid.z = centroid[2];*/

  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(pc, pc2);
  pcl_conversions::fromPCL(pc2, pc_msg);
  //pcl::toROSMsg (pc, pc2);
  s.points.push_back (pc_msg);

  s.holes.push_back(false);
}

void
fourthShape (cob_3d_mapping_msgs::Shape& s, Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f x_axis, Eigen::Vector3f color)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pc;
  sensor_msgs::PointCloud2 pc_msg;
  /*
   Eigen::Vector3f v2 (1, 0, 0), v3 (0, 0, 0), v4;
   //cout << " v1 : \n" << v1 << endl;
   //cout<< " v1 orthognal : \n"<<v1.unitOrthogonal()<<endl;
   //v1.normalize();
   //cout<< " v1 norm : "<<v1<<endl;
   v2 = v1.unitOrthogonal ();
   //cout << " v2 : \n" << v2 << endl;
   //cout<< " v1 euler : \n"<<v1.eulerAngles(v1.de,1,2)<<endl;
   v3 = v2.unitOrthogonal ();
   //cout << " v3 : \n" << v3 << endl;

   v4 = v3.unitOrthogonal ();
   //cout << " v4 : \n" << v3 << endl;
   //sixth shape
   */

  s.id = 3;
  s.params[0] = normal[0];
  s.params[1] = normal[1];
  s.params[2] = normal[2];
  s.params[3] = origin.dot(normal);

  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, origin, t);
  tf::poseEigenToMsg(t.inverse().cast<double>(), s.pose);

  s.color.r = color[0];
  s.color.g = color[1];
  s.color.b = color[2];
  s.color.a = 1;
  /*
   int a = 4;

   for (int i = 1; i < 4; i++)
   {
   if (i % 3 == 0)
   {
   pt.x = a * i * v2[0];
   pt.y = i * v2[1];
   }
   else
   {
   if (i % 2 == 0)
   pt.x = i * v2[0];
   else
   pt.y = a * i * v2[1];
   }
   pt.z = v2[2];
   pc.push_back (pt);
   cout << " pt : \n" << pt << endl;
   }
   */
  //int x = v2[0], y = v2[1], z = v2[2];
  int x = 1, y = 0, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  //x = v3[0], y = v3[1], z = v3[2];
  x = 0, y = 1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  //x = v4[0], y = v4[1], z = v4[2];
  x = 0, y = 0, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  /*
   x = 1, y = 0, z = 1;
   pt.x = x;
   pt.y = y;
   pt.z = z;
   pc.push_back (pt);

   x = v2[0], y = v2[1], z = v2[2];
   pt.x = x;
   pt.y = y;
   pt.z = z;
   pc.push_back (pt);
   */
  //Eigen::VectorXf centroid;
  //pcl::computeNDCentroid (pc, centroid);
  //std::cout << " centroid : \n" << centroid << std::endl;
  //std::cout << " point cloud : \n" << pc << std::endl;
  /*s.centroid.x = centroid[0];
  s.centroid.y = centroid[1];
  s.centroid.z = centroid[2];*/
  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(pc, pc2);
  pcl_conversions::fromPCL(pc2, pc_msg);
  //pcl::toROSMsg (pc, pc2);
  s.points.push_back (pc_msg);

  s.holes.push_back(false);
}

void
fifthShape (cob_3d_mapping_msgs::Shape& s, Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f x_axis, Eigen::Vector3f color)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pc;
  sensor_msgs::PointCloud2 pc_msg;

  s.id = 4;

  s.params[0] = normal[0];
  s.params[1] = normal[1];
  s.params[2] = normal[2];
  s.params[3] = origin.dot(normal);

  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, origin, t);
  tf::poseEigenToMsg(t.inverse().cast<double>(), s.pose);

  s.color.r = color[0];
  s.color.g = color[1];
  s.color.b = color[2];
  s.color.a = 1;
/*
  Eigen::Vector3f v2 (-normal[0], normal[1], -normal[2]), v3 (normal[0], -normal[1], normal[2]), v4 (normal[0], normal[1], -normal[2]);
  cout << " normal : \n" << normal << endl;
  cout << " v2 : \n" << v2 << endl;
  cout << " v3 : \n" << v3 << endl;
  cout << " v4 : \n" << v4 << endl;
  //cout<< " v1 orthognal : \n"<<v1.unitOrthogonal()<<endl;
  //v1.normalize();
  //cout<< " v1 norm : "<<v1<<endl;
  //v2 = v1.unitOrthogonal ();
  cout << " v2.dot (normal) : \n" << v2.dot (normal) << endl;
  cout << " v3.dot (normal) : \n" << v3.dot (normal) << endl;
  cout << " v4.dot (normal) : \n" << v4.dot (normal) << endl;

  //cout<< " v1 euler : \n"<<v1.eulerAngles(v1.de,1,2)<<endl;
  //v3 = v2.unitOrthogonal ();
  //cout << " v3 : \n" << v3 << endl;

  //v4 = v3.unitOrthogonal ();
  //cout << " v4 : \n" << v3 << endl;
  //sixth shape
*/
  float x = 0 , y = -1, z = 0;

  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 1 , y = -1, z = 0;

  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 1.5 , y = -1, z = 0;

  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);


  x = 0.8 , y = -1, z = 0;

  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 0.5 , y = -1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 0 , y = -1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = -1 , y = -1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 0 , y = -1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = -0.5 , y = -1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);
  //Eigen::VectorXf centroid;
  //pcl::computeNDCentroid (pc, centroid);
  //std::cout << " centroid : \n" << centroid << std::endl;

  /*s.centroid.x = centroid[0];
  s.centroid.y = centroid[1];
  s.centroid.z = centroid[2];*/
  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(pc, pc2);
  pcl_conversions::fromPCL(pc2, pc_msg);
  //pcl::toROSMsg (pc, pc2);
  s.points.push_back (pc_msg);

  s.holes.push_back(false);
}

void
sixthShape (cob_3d_mapping_msgs::Shape& s, Eigen::Vector3f origin, Eigen::Vector3f normal, Eigen::Vector3f x_axis, Eigen::Vector3f color)
{

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pc;
  sensor_msgs::PointCloud2 pc2;

  s.id = 5;

  s.params[0] = normal[0];
  s.params[1] = normal[1];
  s.params[2] = normal[2];
  s.params[3] = origin.dot(normal);

  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, origin, t);
  tf::poseEigenToMsg(t.inverse().cast<double>(), s.pose);

  s.color.r = color[0];
  s.color.g = color[1];
  s.color.b = color[2];
  s.color.a = 1;

  int x = 2, y = 0, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 2, y = 1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 2, y = 1, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  x = 2, y = 0, z = 0;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pc.push_back (pt);

  s.holes.push_back(false);
}

int
main (int argc, char **argv)
{

  ros::init (argc, argv, "test_shape_array");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cob_3d_mapping_msgs::ShapeArray> ("/geometry_map/map_array", 1);

  ROS_INFO("\nPublishing shape_array...........");
  ros::Rate loop_rate (1);
  loop_rate.sleep ();
  uint32_t seq = 0;

  //while (ros::ok ())
  {
    cob_3d_mapping_msgs::ShapeArray sa;
    sa.header.frame_id = "/map";
    sa.header.stamp = ros::Time::now ();
    sa.header.seq = seq++;

    cob_3d_mapping_msgs::Shape s;
    s.params.resize (4);

    s.header.frame_id = "/map";
    s.header.stamp = sa.header.stamp;

    s.type = cob_3d_mapping_msgs::Shape::POLYGON;
    //s.holes.push_back (false);

    Eigen::Vector3f normal (0, 0, 1);
    Eigen::Vector3f origin (0, 0, 0);
    Eigen::Vector3f x_axis (1, 0, 0);
    Eigen::Vector3f color (1, 0, 0);

    firstShape (s, origin, normal, x_axis, color);
    sa.shapes.push_back (s);
    s.points.clear ();
    s.holes.clear();

    color[0] = 0;
    color[1] = 1;
    color[2] = 0;
    origin(2) = 1;

    secondShape (s, origin, normal, x_axis, color);
    sa.shapes.push_back (s);
    s.points.clear ();
    s.holes.clear();

    color[0] = 1;
    color[1] = 1;
    color[2] = 0;
    origin(1) = 1;
    normal = Eigen::Vector3f(0,1,1);
    normal.normalize();

    thirdShape (s, origin, normal, x_axis, color);
    sa.shapes.push_back (s);
    s.points.clear ();
    s.holes.clear();

    normal[0] = 0.707;
    normal[1] = 0.707;
    normal[2] = 0.707;
    origin = Eigen::Vector3f(-1,-1,-1);

    color[0] = 0;
    color[1] = 0;
    color[2] = 1;

    fourthShape (s, origin, normal, x_axis, color);
    sa.shapes.push_back (s);
    s.points.clear ();
    s.holes.clear();

    /*normal[0] = 0;
    normal[1] = 1;
    normal[2] = 0;

    color[0] = 1;
    color[1] = 1;
    color[2] = 1;
    fifthShape (s, origin, normal, x_axis, color);
    sa.shapes.push_back (s);
    s.points.clear ();

    normal[0] = 1;
    normal[1] = 0;
    normal[2] = 0;

    color[0] = 0;
    color[1] = 1;
    color[2] = 1;
    sixthShape (s, origin, normal, x_axis, color);
    sa.shapes.push_back (s);
    s.points.clear ();*/

    pub.publish (sa);
    loop_rate.sleep ();
    ros::spin ();
    //loop_rate.sleep ();
  }

  return 0;
}

