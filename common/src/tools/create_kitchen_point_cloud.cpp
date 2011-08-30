/*
 * create_kitchen_point_cloud.cpp
 *
 *  Created on: 15.06.2011
 *      Author: goa
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <fstream>
#include <pcl/filters/voxel_grid.h>


//create edge cloud
int main(int argc, char** argv)
{
  double incr = 0.02;
  pcl::PointCloud<pcl::PointXYZRGB> whole_kitchen;
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
  pcl::copyPointCloud(kitchen, whole_kitchen);
  kitchen.points.clear();
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
  //back
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
  //front
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_lf.pcd", kitchen, true);
  kitchen.points.clear();
  std::ofstream plane_file;
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_lf.pl");
  plane_file << 1 << " " << 0 << " " << 0 << " " << -x_end;
  plane_file.close();
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_ll.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_ll.pl");
  plane_file << 0 << " " << -1 << " " << 0 << " " << y_start;
  plane_file.close();
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_lr.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_lr.pl");
  plane_file << 0 << " " << 1 << " " << 0 << " " << -y_end;
  plane_file.close();

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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
  //back
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
  //front
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_rf.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_rf.pl");
  plane_file << 1 << " " << 0 << " " << 0 << " " << -x_end;
  plane_file.close();
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_rl.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_rl.pl");
  plane_file << 0 << " " << -1 << " " << 0 << " " << y_start;
  plane_file.close();
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_rr.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_rr.pl");
  plane_file << 0 << " " << 1 << " " << 0 << " " << -y_end;
  plane_file.close();

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
  kitchen.points.clear();
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_mt.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_mt.pl");
  plane_file << 0 << " " << 0 << " " << 1 << " " << -z_end;
  plane_file.close();
  //back
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
  //front
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_mf.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_mf.pl");
  plane_file << 1 << " " << 0 << " " << 0 << " " << -x_end;
  plane_file.close();
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();
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
  whole_kitchen+=kitchen;
  kitchen.points.clear();

  //wall behind kitchen
  z_start = 0;
  z_end = 2.5;
  y_start = -2.1;
  y_end = 2.316;
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_wb.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_wb.pl");
  plane_file << 1 << " " << 0 << " " << 0 << " " << -x_end;
  plane_file.close();

  //wall right of kitchen
  z_start = 0;
  z_end = 2.5;
  x_start = -3.531;
  x_end = -2;
  for(double z = z_start; z <= z_end; z+=incr)
  {
    for(double x = x_start; x <= x_end; x+=incr)
    {
      pcl::PointXYZRGB p;
      p.x = x;
      p.y = 2.316;
      p.z = z;
      kitchen.push_back(p);
    }
  }
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_wr.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_wl.pl");
  plane_file << 0 << " " << -1 << " " << 0 << " " << y_start;
  plane_file.close();

  //floor
  x_start = -3.531;
  x_end = 0;
  y_start = -2.1;
  y_end = 2.316;
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
  whole_kitchen+=kitchen;
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_f.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_f.pl");
  plane_file << 1 << " " << 0 << " " << 0 << " " << -x_end;
  plane_file.close();
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(whole_kitchen.makeShared());
  voxel.setLeafSize(incr,incr,incr);
  pcl::PointCloud<pcl::PointXYZRGB> whole_kitchen_vox;
  voxel.filter(whole_kitchen_vox);
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/whole_kitchen.pcd", whole_kitchen_vox, true);
}

