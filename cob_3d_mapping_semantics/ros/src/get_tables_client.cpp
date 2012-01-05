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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_semantics
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 01/2012
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

//standard includes
#include <iostream>
#include <fstream>
#include <sstream>

//ros includes
#include "ros/ros.h"
#include <rosbag/bag.h>

// ROS message includes
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class TablesClient
{
public:
  // Constructor
  TablesClient ()

  {
    get_tables_client_ = nh.serviceClient<cob_3d_mapping_msgs::GetObjectsOfClass> ("get_objects_of_class");
    pc2_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/get_tables_client/point_cloud2", 100);
    ros::param::param ("~file_path", file_path_, std::string ("/home/goa-wq/tmp/"));
    ros::param::param ("~save_txt_file", save_txt_file_, true);
    ros::param::param ("~save_pcd_file", save_pcd_file_, true);
    ros::param::param ("~save_bag_file", save_bag_file_, true);

    std::cout << "ROS_PARAM: file_path = " << file_path_ << std::endl;
    ROS_INFO("ROS_PARAM: save_txt_file = %d",save_txt_file_);
    ROS_INFO("ROS_PARAM: save_pcd_file = %d",save_pcd_file_);
    ROS_INFO("ROS_PARAM: save_bag_file = %d",save_bag_file_);
  }

  // Destructor
  ~TablesClient ()
  {
    /// void
  }

  /**
   * @brief calls "get_objects_of_class" service
   *
   * @return nothing
   */
  void
  callService ()
  {
    ROS_INFO("waiting for server to start");
    ros::service::waitForService ("get_objects_of_class");
    ROS_INFO("Service call finished");
    //build message
    cob_3d_mapping_msgs::GetObjectsOfClassRequest req;
    cob_3d_mapping_msgs::GetObjectsOfClassResponse res;
    get_tables_client_.call (req, res);
    saveTables (res);
  }

  /**
   * @brief Save table objects to a file
   *
   * @param res Service response
   *
   * @return nothing
   */
  void
  saveTables (cob_3d_mapping_msgs::GetObjectsOfClass::Response &res)
  {
    ROS_INFO("Table objects received : %d",res.objects.shapes.size ());
    if (save_txt_file_ || save_pcd_file_)
    {
      for (unsigned int i = 0; i < res.objects.shapes.size (); i++)
      {
        if (save_txt_file_ == true)
        {
          std::ofstream object_file;
          std::stringstream ss;
          ROS_INFO("Saving Table object: table_ %d",i);
          ss << file_path_ << "table_" << i << ".txt";
          object_file.open (ss.str ().c_str ());
          object_file << "#Table Points\n";
          object_file << res.objects.shapes[i] << "\n";
          object_file.close ();
        }
        if (save_pcd_file_ == true)
        {
          for (unsigned int j = 0; j < res.objects.shapes[i].points.size (); j++)
          {
            std::stringstream ss1;
            ss1 << file_path_ << "shape_" << i << "_" << j << ".pcd";
            pcl::PointCloud<pcl::PointXYZ> map;
            pcl::fromROSMsg (res.objects.shapes[i].points[j], map);
            pcl::io::savePCDFile (ss1.str (), map, false);
          }
        }
      }
    }

    if (save_bag_file_ == true)
    {
      cob_3d_mapping_msgs::ShapeArray sa;// = res.objects.shapes;
      sa.header.stamp = ros::Time (1.0);

      sa.header.frame_id = "/map";
      for (unsigned int i = 0; i < res.objects.shapes.size (); i++)
      {
        sa.shapes.push_back (res.objects.shapes[i]);
        /*
        sensor_msgs::PointCloud2 pc2;
        pc2 = res.objects.shapes[i].points[0];
        pc2.header.frame_id = "/map";
        while(1)
        pc2_pub_.publish(pc2);
        */
      }

      rosbag::Bag bag;
      std::stringstream ss1;
      ss1 << file_path_ << "table.bag";
      bag.open (ss1.str().c_str(), rosbag::bagmode::Write);
      //bag.write("shape_array",ros::Time::now(),sa);
      bag.write ("/get_tables_client/shape_array", sa.header.stamp, sa);
      bag.close ();
      std::cout << sa.header.stamp << std::endl;
    }

  }

  ros::NodeHandle nh;

protected:
  ros::ServiceClient get_tables_client_;
  ros::Publisher pc2_pub_;

  std::string file_path_;
  bool save_txt_file_;
  bool save_pcd_file_;
  bool save_bag_file_;

};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "client");

  TablesClient client;
  client.callService ();

  ros::spin ();
  return 0;
}

