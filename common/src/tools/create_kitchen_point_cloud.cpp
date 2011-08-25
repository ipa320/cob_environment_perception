/*
 * create_kitchen_point_cloud.cpp
 *
 *  Created on: 15.06.2011
 *      Author: goa
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"


//create edge cloud
int main(int argc, char** argv)
{
  double incr = 0.02;
  pcl::PointCloud<pcl::PointXYZRGB> kitchen;
  //left shelf
  double x_start = -3.201 - 0.33;
  double x_end = -3.201 + 0.33;
  double y_start = -1.155 - 0.31;
  double y_end = -1.155 + 0.31;
  double z_start = 0.72 - 0.72;
  double z_end = 0.72 + 0.72;
  //bottom
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = z_start;
      kitchen.push_back(p);
    }
  }
  //top
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = z_end;
      kitchen.push_back(p);
    }
  }
  //front
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x_start;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //back
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x_end;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //left
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y_start;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //right
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y_end;
      p.z = z;
      kitchen.push_back(p);
    }
  }

  //right shelf
  x_start = -3.201 - 0.33;
  x_end = -3.201 + 0.33;
  y_start = 1.265 - 0.31;
  y_end = 1.265 + 0.31;
  z_start = 0.72 - 0.72;
  z_end = 0.72 + 0.72;
  //bottom
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = z_start;
      kitchen.push_back(p);
    }
  }
  //top
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = z_end;
      kitchen.push_back(p);
    }
  }
  //front
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x_start;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //back
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x_end;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //left
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y_start;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //right
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y_end;
      p.z = z;
      kitchen.push_back(p);
    }
  }

  //middle part
  x_start = -3.201 - 0.33;
  x_end = -3.201 + 0.33;
  y_start = 0.055 - 0.9;
  y_end = 0.055 + 0.9;
  z_start = 0.45 - 0.45;
  z_end = 0.45 + 0.45;
  //bottom
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = z_start;
      kitchen.push_back(p);
    }
  }
  //top
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = z_end;
      kitchen.push_back(p);
    }
  }
  //front
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x_start;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //back
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x_end;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //left
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y_start;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  //right
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y_end;
      p.z = z;
      kitchen.push_back(p);
    }
  }

  //wall behind kitchen
  z_start = 0;
  z_end = 2.5;
  y_start = -2.1;
  y_end = 2;
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = -3.531;
      p.y = y;
      p.z = z;
      kitchen.push_back(p);
    }
  }

  //floor
  x_start = -3.531;
  x_end = 0;
  y_start = -2.1;
  y_end = 2;
  for(double x = x_start; x <= x_end; x+=incr)
  {
    for(double y = y_start; y <= y_end; y+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = y;
      p.z = 0;
      kitchen.push_back(p);
    }
  }
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_gt.pcd", kitchen, true);
}

