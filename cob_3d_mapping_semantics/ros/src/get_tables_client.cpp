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
 *  ROS package name: cob_3d_mapping_semantics
 *
 * \author
 *  Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 01/2012
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

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
    //build message
    cob_3d_mapping_msgs::GetObjectsOfClassRequest req;
    cob_3d_mapping_msgs::GetObjectsOfClassResponse res;
    get_tables_client_.call (req, res);
    ROS_INFO("Service call finished");
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
    ROS_INFO("Table objects received : %d", (int)res.objects.shapes.size ());
    if (save_txt_file_ || save_pcd_file_)
    {
      for (unsigned int i = 0; i < res.objects.shapes.size (); i++)
      {
        if (save_txt_file_ == true && res.objects.shapes.size() > 0)
        {
          std::ofstream object_file;
          std::stringstream ss;
          ROS_INFO("Saving Table object: table_ %d",i);
          ss << file_path_ << "table_" << i << ".txt";
          object_file.open (ss.str ().c_str ());
          object_file << "#Table Points\n";
          object_file << res.objects.shapes[i] << "\n";
          object_file.close ();
          ROS_INFO("Text file saved");
        }
        if (save_pcd_file_ == true && res.objects.shapes.size() > 0)
        {
          for (unsigned int j = 0; j < res.objects.shapes[i].points.size (); j++)
          {
            ROS_INFO("Saving Table object: table_%d_%d",i,j);
            std::stringstream ss1;
            ss1 << file_path_ << "table_" << i << "_" << j << ".pcd";
            pcl::PointCloud<pcl::PointXYZ> p;
            pcl::PCLPointCloud2 p2;
            pcl_conversions::toPCL(res.objects.shapes[i].points[j], p2);
            pcl::fromPCLPointCloud2(p2, p);
            //pcl::fromROSMsg (res.objects.shapes[i].points[j], p);
            pcl::io::savePCDFile (ss1.str (), p, false);
            ROS_INFO("PCD file saved");
          }
        }

      }
    }

    if (save_bag_file_ == true && res.objects.shapes.size() > 0)
    {
      cob_3d_mapping_msgs::ShapeArray sa;// = res.objects.shapes;
      sa.header.stamp = ros::Time (1.0);

      sa.header.frame_id = "/map";
      for (unsigned int i = 0; i < res.objects.shapes.size (); i++)
      {
        sa.shapes.push_back (res.objects.shapes[i]);
        /*
        for (unsigned int j = 0; j < res.objects.shapes[i].points.size (); j++)
        {
          sensor_msgs::PointCloud2 pc2;
          pc2 = res.objects.shapes[i].points[j];
          pc2.header.frame_id = "/map";
          pc2_pub_.publish(pc2);
          ros::Duration(10).sleep();
        }
        */
      }

    rosbag::Bag bag;
    std::stringstream ss1;
    ss1 << file_path_ << "table.bag";
    bag.open (ss1.str().c_str(), rosbag::bagmode::Write);
    bag.write ("/get_tables_client/shape_array", sa.header.stamp, sa);
    bag.close ();
    ROS_INFO("BAG file saved");
  }
    else
      ROS_INFO("No table object : BAG file not saved");

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

  //ros::spin ();
  return 0;
}

