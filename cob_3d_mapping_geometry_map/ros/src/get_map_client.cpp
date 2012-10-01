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
 *  ROS package name: cob_3d_mapping_geometry_map
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

// ROS message includes
#include <cob_3d_mapping_msgs/GetGeometricMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <fstream>

void writeToFile(cob_3d_mapping_msgs::Shape& s, std::ofstream& fs)
{
  //static int ctr=0;
  //std::stringstream ss;
  //ss << "/home/goa/pcl_daten/kitchen_kinect/polygons/polygon_" << ctr << ".txt";
  //std::ofstream myfile;
  //myfile.open (ss.str().c_str());
  fs << (int)s.type << "\n";
  for(unsigned int i=0; i<s.params.size(); i++)
    fs << s.params[i] << " ";

  //myfile.close();

}


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
  cob_3d_mapping_msgs::GetGeometricMapRequest req;
  cob_3d_mapping_msgs::GetGeometricMapResponse resp;

  if (ros::service::call("/geometry_map/get_map", req,resp))
  {
    ROS_INFO("Service call finished.");
  }
  else
  {
    ROS_INFO("Service call failed.");
    return 0;
  }

  // maybe better dump to bag file?
  /*for(unsigned int i=0; i<resp.shapes.size(); i++)
  {
    std::stringstream ss;
    ss << "shape_" << i << ".txt";
    std::ofstream fs;
    fs.open (ss.str().c_str());
    writeToFile(resp.shapes[i], fs);
    fs.close();
    for(unsigned int j=0; j<resp.shapes[i].points.size(); j++)
    {
      std::stringstream ss1;
      ss1 << "shape_" << i << "_" << j << ".pcd";
      pcl::PointCloud<pcl::PointXYZ> map;
      pcl::fromROSMsg(resp.shapes[i].points[j], map);
      pcl::io::savePCDFile(ss1.str(),map,false);
    }
  }*/
  //cob_3d_mapping_msgs::ShapeArray& sa;
  //cob_3d_mapping_msgs::ShapeArray sa = resp.shapes.shapes;
  //sa.header.stamp = ros::Time(1.0);
  //std::cout << sa.header.stamp << std::endl;
  //sa.header.frame_id = "/map";
  /*for(unsigned int i=0; i<resp.shapes.size(); i++)
    sa.shapes.push_back(resp.shapes[i]);*/

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Write);
  bag.write("geometry_map", resp.map.header.stamp, resp.map);

  bag.close();

  //exit
  return 0;
}

