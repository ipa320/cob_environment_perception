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
#include <boost/foreach.hpp>

// ROS message includes
#include <cob_3d_mapping_msgs/ShapeArray.h>

std::string file_path;


int main (int argc, char **argv)
{
  ros::init(argc, argv, "geometry_map_node");
  ros::NodeHandle nh;

  ros::param::get("~file_path", file_path);
  file_path="/home/develop/plane_sim_geo.bag";
  
  std::vector< std::vector<cob_3d_mapping_msgs::Shape> > map;
  map.resize(100);
  std::vector< std::vector<cob_3d_mapping_msgs::Shape> > seg;
  seg.resize(100);
  
  rosbag::Bag bag;
  bag.open(file_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/geometry_map/out_shape_array"));
  topics.push_back(std::string("/geometry_map/map_array"));
  //rosbag::View view(bag, rosbag::TopicQuery("/geometry_map/map_array"));
  rosbag::View view(bag, rosbag::TopicQuery("/geometry_map/map_array"));
  
  unsigned int max_id=0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    //rosbag::MessageInstance m = *(view.begin());
    cob_3d_mapping_msgs::ShapeArray::Ptr sa = m.instantiate<cob_3d_mapping_msgs::ShapeArray>();
    for(unsigned int i=0; i<sa->shapes.size(); i++)
    {
        //cob_3d_mapping_msgs::Shape::Ptr s_ptr(&sa->shapes[i]);
        map[sa->shapes[i].id].push_back(sa->shapes[i]);
        if(sa->shapes[i].id>max_id) max_id=sa->shapes[i].id;
    }
  }
  map.resize(max_id+1);
  
  rosbag::View view2(bag, rosbag::TopicQuery("/geometry_map/out_shape_array"));
  max_id=0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    //rosbag::MessageInstance m = *(view.begin());
    cob_3d_mapping_msgs::ShapeArray::Ptr sa = m.instantiate<cob_3d_mapping_msgs::ShapeArray>();
    for(unsigned int i=0; i<sa->shapes.size(); i++)
    {
        //cob_3d_mapping_msgs::Shape::Ptr s_ptr(&sa->shapes[i]);
        seg[sa->shapes[i].id].push_back(sa->shapes[i]);
        if(sa->shapes[i].id>max_id) max_id=sa->shapes[i].id;
    }
  }
  seg.resize(max_id+1);
  bag.close();
  
  std::cout << seg.size() << "," << map.size() << std::endl;
  
  for(unsigned int i=0; i<map.size(); i++)
  {
    if(map[i].size() == 0 || seg[i].size() == 0) continue;
    std::cout << i << ":" << map[i][0].params[0] << "," << map[i][0].params[1] << "," << map[i][0].params[2] << "," << map[i][0].params[3] << std::endl << std::endl;
    for(unsigned int j=0; j < seg[i].size(); j++)
        std::cout << i << ":" << seg[i][j].params[0] << "," << seg[i][j].params[1] << "," << seg[i][j].params[2] << "," << seg[i][j].params[3] << std::endl;
    std::cout << std::endl;
  }


  return 0;
}
