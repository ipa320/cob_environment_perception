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

#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include <cob_3d_mapping_msgs/CurvedPolygonArray.h>

#include <actionlib/server/simple_action_server.h>
#include <cob_3d_segmentation/ObjectWatchGoal.h>
#include <cob_3d_segmentation/ObjectWatchFeedback.h>
#include <cob_3d_segmentation/ObjectWatchAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <cob_3d_segmentation/point_types.h>


int main(int argc, char **argv) {
  if(argc<3) {
    PCL_ERROR ("Arguments: [input pointcloud *.pcd] [output file]\n");
    return (-1);
  }

  ros::init(argc, argv, "qppf");
  const std::string ifn=argv[1];
  std::ofstream fos(argv[2]);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ifn, *cloud) == -1) //* load the file
  {
    PCL_ERROR (("Couldn't read file "+ifn+" \n").c_str());
    return (-1);
  }

  Segmentation::Segmentation_QuadRegression<pcl::PointXYZ, pcl::PointXYZRGB, Segmentation::QPPF::QuadRegression<2, pcl::PointXYZ, Segmentation::QPPF::CameraModel_Kinect<pcl::PointXYZ> > > seg;
  seg.setInputCloud(cloud);
  seg.compute();
  seg.serialize(fos);

  return 0;
}

