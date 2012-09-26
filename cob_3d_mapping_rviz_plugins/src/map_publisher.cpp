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
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif
#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/Shape.h>

#include <ros/time.h>

#include <boost/shared_ptr.hpp>


int main(int argc, char **argv) {
  



  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cob_3d_mapping_msgs::ShapeArray>("/segmentation/shape_array",1);

  ros::Rate loop_rate(1);

  int number_shapes=10;
  if(argc==2)
  {
      number_shapes = atoi(argv[1]);
      std::cout<<"Publishing map with "<<number_shapes<<" shapes.\n";
  }
  else
  {
      std::cout<<"WARNING: Use number of shapes in map as input argument.(Using default value 10) \n";
  }


  float x_shift,y_shift,z_shift;

  x_shift=-0.5;
  y_shift=-0.5;
  z_shift=-0.5;


    int runner = 1;
  //while(ros::ok()) {
      for(int k=0;k<2;++k){
          
    cob_3d_mapping_msgs::ShapeArray sa;
    sa.header.frame_id="/map";

    for(int i =0 ;i<number_shapes;i++)
    {

// Transform shape with pcl:: transformation
    Eigen::Affine3f transform ;
    Eigen::Vector3f u1,u2,o;

    u1<<0,1,0;
    u2<<0,0,1;
    o<<i*x_shift,i*y_shift,i*z_shift;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(u1,u2,o,transform);
     
    
    cob_3d_mapping_msgs::Shape s;
    //transform parameters
    Eigen::Vector3f normal,centroid;
    double d;

    normal << 0, 0, 1;
    normal= transform.rotation() * normal;

    centroid << 1, 1, 1;
    centroid =transform * centroid;

    d= 0;
    d=fabs(centroid.dot(normal));


    // put params to shape msg
    s.params.push_back(normal[0]);
    s.params.push_back(normal[1]);
    s.params.push_back(normal[2]);
    s.params.push_back(d);
    s.centroid.x = centroid[0];
    s.centroid.y = centroid[1];
    s.centroid.z = centroid[2];
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


    
    //transform poincloud
    pcl::getTransformedPointCloud(pc,transform,pc);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc,pc2);
    s.points.push_back(pc2);
    s.holes.push_back(false);
    sa.shapes.push_back(s);
  }

    pub.publish(sa);

    ros::spinOnce();
    loop_rate.sleep();
    runner++;
 }

  return 0;
}

