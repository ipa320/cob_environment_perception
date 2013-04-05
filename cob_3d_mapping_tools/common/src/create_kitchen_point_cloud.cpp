/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
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
  pcl::PointCloud<pcl::PointXYZRGB> front;
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
  pcl::copyPointCloud(kitchen, front);
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
  plane_file << 0 << " " << -1 << " " << 0 << " " << y_end;
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
  front+=kitchen;
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
  plane_file << 0 << " " << 1 << " " << 0 << " " << -y_start;
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
  front+=kitchen;
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
  x_end = -3.531;
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
  y_start = 2.281;
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
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_wr.pcd", kitchen, true);
  kitchen.points.clear();
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_wr.pl");
  plane_file << 0 << " " << 1 << " " << 0 << " " << -y_start;
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
  plane_file << 0 << " " << 0 << " " << 1 << " " << -x_end;
  plane_file.close();
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(whole_kitchen.makeShared());
  voxel.setLeafSize(incr,incr,incr);
  pcl::PointCloud<pcl::PointXYZRGB> whole_kitchen_vox;
  voxel.filter(whole_kitchen_vox);
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/whole_kitchen.pcd", whole_kitchen_vox, true);
  pcl::io::savePCDFile ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_front.pcd", front, true);
  plane_file.open ("/home/goa/pcl_daten/kitchen_ground_truth/kitchen_front.pl");
  plane_file << 1 << " " << 0 << " " << 0 << " " << 2.871;
  plane_file.close();
}

