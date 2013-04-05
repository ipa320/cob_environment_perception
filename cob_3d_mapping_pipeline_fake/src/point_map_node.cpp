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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_pipeline_fake
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>

std::string file_path;
sensor_msgs::PointCloud2::ConstPtr pc;

bool
getMap(cob_3d_mapping_msgs::GetPointMap::Request &req,
       cob_3d_mapping_msgs::GetPointMap::Response &res)
{
  res.map = *pc;
  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "point_map_node");
  ros::NodeHandle nh;

  ros::param::get("~file_path", file_path);
  rosbag::Bag bag;
  bag.open(file_path, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("/point_map/map"));
  rosbag::MessageInstance m = *(view.begin());
  pc = m.instantiate<sensor_msgs::PointCloud2>();
  bag.close();

  ros::ServiceServer get_map_server = nh.advertiseService("get_map", &getMap);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("map",1);
  ros::Duration(0.5).sleep();
  pub.publish(*pc);
  ros::spin();

  return 0;
}
