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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2011
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

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

// pcl includes
#include <pcl/point_types.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/pcd_io.h>

// cob_env_model includes
#include <cob_env_model/cpc_point.h>
#include <cob_env_model/cob_filters/amplitude_filter.h>
#include <cob_env_model/cob_filters/impl/amplitude_filter.hpp>

//######################
//#### nodelet class####
class AmplitudeFilter : public pcl_ros::PCLNodelet
{
public:
  // Constructor
  AmplitudeFilter () :
    t_check (0)
  {
    //
  }

  // Destructor
  ~ AmplitudeFilter ()
  {
    /// void
  }

  void
  onInit ()
  {
    PCLNodelet::onInit ();
    n_ = getNodeHandle ();

    point_cloud_sub_ = n_.subscribe ("point_cloud2", 1, &AmplitudeFilter::PointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2> ("point_cloud2_filtered", 1);

    n_.param ("/amplitude_filter_nodelet/amplitude_min_threshold", lim_min_, 2000);
    //std::cout << "amplitude_min_threshold: " << lim_min_<< std::endl;
    n_.param ("/amplitude_filter_nodelet/amplitude_max_threshold", lim_max_, 60000);
    //std::cout << "amplitude_max_threshold: " << lim_max_<< std::endl;
  }

  void
  PointCloudSubCallback (const pcl::PointCloud<CPCPoint>::Ptr pc)
  {
    //ROS_INFO("PointCloudSubCallback");
    cob_env_model::AmplitudeFilter<CPCPoint> filter;
    pcl::PointCloud<CPCPoint>::Ptr cloud_filtered (new pcl::PointCloud<CPCPoint> ());
    filter.setInputCloud (pc);
    filter.setFilterLimits (lim_min_, lim_max_);
    //std::cout<<" min_limit :"<<filter.getFilterMinLimit()<<std::endl;
    //std::cout<<" max_limit :"<<filter.getFilterMaxLimit()<<std::endl;
    filter.applyFilter (*cloud_filtered);
    point_cloud_pub_.publish (cloud_filtered);
    if (t_check == 0)
    {
      ROS_INFO("Time elapsed (Amplitude_Filter) : %f", t.elapsed());
      t.restart ();
      t_check = 1;
    }
  }

  // Test specialized template for sensor_msgs::PointCloud2 point_type
/*
  void
  PointCloudSubCallback (sensor_msgs::PointCloud2::ConstPtr pc)
  {
    //ROS_INFO("PointCloudSubCallback");
    cob_env_model::AmplitudeFilter<sensor_msgs::PointCloud2> filter;
    sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
    filter.setInputCloud (pc);
    filter.setFilterLimits (lim_min_, lim_max_);
    //std::cout<<" min_limit :"<<filter.getFilterMinLimit()<<std::endl;
    //std::cout<<" max_limit :"<<filter.getFilterMaxLimit()<<std::endl;
    filter.applyFilter (*cloud_filtered);
    point_cloud_pub_.publish (cloud_filtered);
    if (t_check == 0)
    {
      ROS_INFO("Time elapsed (Amplitude_Filter) : %f", t.elapsed());
      t.restart ();
      t_check = 1;
    }
  }
*/
  ros::NodeHandle n_;
  boost::timer t;

protected:
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;

  int lim_min_;
  int lim_max_;
  bool t_check;
};

PLUGINLIB_DECLARE_CLASS(cob_env_model, AmplitudeFilter, AmplitudeFilter, nodelet::Nodelet)

