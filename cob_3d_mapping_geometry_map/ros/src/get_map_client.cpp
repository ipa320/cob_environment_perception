/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_geometry_map
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 12/2011
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>

// ROS message includes
#include <cob_3d_mapping_msgs/GetGeometryMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main (int argc, char **argv)
{
  if(argc<1) {
    ROS_ERROR("Please specify output file\nrosrun cob_3d_mapping_geometry_map get_map_client myfile.bag");
    return -1;
  }
  ros::init(argc, argv, "get_geoemtry_map");

  ros::NodeHandle nh;

  ROS_INFO("Waiting for service server to start.");
  ros::service::waitForService("/geometry_map/get_map"); //will wait for infinite time

  ROS_INFO("Server started, polling map.");

  //build message
  cob_3d_mapping_msgs::GetGeometryMapRequest req;
  cob_3d_mapping_msgs::GetGeometryMapResponse resp;

  if (ros::service::call("/geometry_map/get_map", req,resp))
  {
    ROS_INFO("Service call finished.");
  }
  else
  {
    ROS_INFO("Service call failed.");
    return 0;
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Write);
  bag.write("/geometry_map/map_array", resp.map.header.stamp, resp.map);

  bag.close();

  //exit
  return 0;
}

