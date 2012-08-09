/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_pipeline_fake
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 12/2011
 * ToDo:
 *
 *
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
#include <rosbag/view.h>

// ROS message includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/GetGeometricMap.h>

std::string file_path;
cob_3d_mapping_msgs::ShapeArray::ConstPtr sa;

bool
getMap(cob_3d_mapping_msgs::GetGeometricMap::Request &req,
       cob_3d_mapping_msgs::GetGeometricMap::Response &res)
{
  res.map = *sa;
  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "geometry_map_node");
  ros::NodeHandle nh;

  ros::param::get("~file_path", file_path);
  rosbag::Bag bag;
  bag.open(file_path, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("/geometry_map/map_array"));
  rosbag::MessageInstance m = *(view.begin());
  sa = m.instantiate<cob_3d_mapping_msgs::ShapeArray>();
  bag.close();

  ros::ServiceServer get_map_server = nh.advertiseService("get_map", &getMap);
  ros::Publisher pub = nh.advertise<cob_3d_mapping_msgs::ShapeArray>("map_array",1);
  ros::Duration(0.5).sleep();
  pub.publish(*sa);
  ros::spin();

  return 0;
}
