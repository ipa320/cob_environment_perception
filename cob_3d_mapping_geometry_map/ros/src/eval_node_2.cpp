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
#include <boost/foreach.hpp>
#include <Eigen/Core>

// ROS message includes
#include <cob_3d_mapping_msgs/ShapeArray.h>

std::string file_path;
std::vector<cob_3d_mapping_msgs::Shape> map;
std::vector<cob_3d_mapping_msgs::Shape> seg;

void shapeCallback1(const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
{
  for(unsigned int i=0; i<sa->shapes.size(); i++)
  {
      //seg.push_back(sa->shapes[i]);
      Eigen::Vector3f n(sa->shapes[i].params[0],sa->shapes[i].params[1],sa->shapes[i].params[2]);
      Eigen::Vector3f n_true(0,-1,0);
      double dangle = std::acos(n.dot(n_true)/n.norm());
      double dd = sa->shapes[i].params[3];
      std::cout << dangle << "\t" << dd << std::endl;
  }
}

void shapeCallback2(const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
{
  for(unsigned int i=0; i<sa->shapes.size(); i++)
  {
      map.push_back(sa->shapes[i]);
  }
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "geometry_map_node");
  ros::NodeHandle nh;

  ros::param::get("~file_path", file_path);
  file_path="/home/goa/pcl_daten/merge_planes/planes_merged.bag";

  ros::Subscriber seg_sub = nh.subscribe("/geometry_map/map_array", 10, shapeCallback1);
  //ros::Subscriber map_sub = nh.subscribe("shape_array_2", 10, shapeCallback2);

  ros::spin();

  /*std::vector<cob_3d_mapping_msgs::Shape> map;
  //map.resize(100);
  std::vector<cob_3d_mapping_msgs::Shape> seg;
  //seg.resize(100);

  rosbag::Bag bag;
  bag.open(file_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/segmentation/shape_array"));
  topics.push_back(std::string("/geometry_map/map_array"));
  //rosbag::View view(bag, rosbag::TopicQuery("/geometry_map/map_array"));
  rosbag::View view(bag, rosbag::TopicQuery("/geometry_map/map_array"));

  //unsigned int max_id=0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    //rosbag::MessageInstance m = *(view.begin());
    cob_3d_mapping_msgs::ShapeArray::Ptr sa = m.instantiate<cob_3d_mapping_msgs::ShapeArray>();
    for(unsigned int i=0; i<sa->shapes.size(); i++)
    {
        //cob_3d_mapping_msgs::Shape::Ptr s_ptr(&sa->shapes[i]);
        map.push_back(sa->shapes[i]);
        //if(sa->shapes[i].id>max_id) max_id=sa->shapes[i].id;
    }
  }
  //map.resize(max_id+1);

  rosbag::View view2(bag, rosbag::TopicQuery("/segmentation/shape_array"));
  //max_id=0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    //rosbag::MessageInstance m = *(view.begin());
    cob_3d_mapping_msgs::ShapeArray::Ptr sa = m.instantiate<cob_3d_mapping_msgs::ShapeArray>();
    for(unsigned int i=0; i<sa->shapes.size(); i++)
    {
        //cob_3d_mapping_msgs::Shape::Ptr s_ptr(&sa->shapes[i]);
        seg.push_back(sa->shapes[i]);
        //if(sa->shapes[i].id>max_id) max_id=sa->shapes[i].id;
    }
  }
  //seg.resize(max_id+1);
  bag.close();

  std::cout << seg.size() << "," << map.size() << std::endl;
  std::cout << "alpha_seg\td_seg\talpha_map\talpha_seg" << std::endl;

  for(unsigned int i=0; i<seg.size(); i++)
  {
    std::cout << seg[i].params[0] << "," << seg[i].params[1] << "," << seg[i].params[2] <<  "," << seg[i].params[3] << std::endl;
    Eigen::Vector3f n(seg[i].params[0],seg[i].params[1],seg[i].params[2]);
    Eigen::Vector3f n_true(0,-1,0);
    double dangle = std::acos(n.dot(n_true)/n.norm());
    double dd = seg[i].params[3];

    Eigen::Vector3f n_m(map[i].params[0],map[i].params[1],map[i].params[2]);
    double dangle2 = std::acos(n_m.dot(n_true)/n_m.norm());
    double dd2 = map[i].params[3];
    std::cout << dangle << "\t" << dd << "\t" << dangle2 << "\t" << dd2<< std::endl;
  }*/



  return 0;
}
