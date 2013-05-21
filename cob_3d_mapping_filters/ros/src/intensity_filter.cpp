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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_filters
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2011
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
#include <nodelet/nodelet.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

// cob_3d_mapping_filters includes
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_filters/intensity_filter.h>
#include <cob_3d_mapping_filters/impl/intensity_filter.hpp>

//######################
//#### nodelet class####
class IntensityFilter : public nodelet::Nodelet
{
public:
  // Constructor
  IntensityFilter () :
    t_check (0)
  {
    //
  }

  // Destructor
  ~ IntensityFilter ()
  {
    /// void
  }

  void
  onInit ()
  {
    n_ = getNodeHandle ();

    point_cloud_sub_ = n_.subscribe ("point_cloud2", 1, &IntensityFilter::PointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2> ("point_cloud2_filtered", 1);

    n_.param ("/intensity_filter_nodelet/intensity_min_threshold", lim_min_, 2000);
    //std::cout << "intensity_min_threshold: " << lim_min_<< std::endl;
    n_.param ("/intensity_filter_nodelet/intensity_max_threshold", lim_max_, 60000);
    //std::cout << "intensity_threshold: " << lim_<< std::endl;

  }

/*  void
  PointCloudSubCallback (const pcl::PointCloud<CPCPoint>::Ptr pc)
  {
    //ROS_INFO("PointCloudSubCallback");
    cob_3d_mapping_filters::IntensityFilter<CPCPoint> filter;
    pcl::PointCloud<CPCPoint>::Ptr cloud_filtered (new pcl::PointCloud<CPCPoint> ());
    filter.setInputCloud (pc);
    filter.setFilterLimit (lim_);
    //std::cout << "  Filter Limit: "<<filter.getFilterLimit()<< std::endl;
    filter.applyFilter (*cloud_filtered);

    point_cloud_pub_.publish (cloud_filtered);
    if (t_check == 0)
    {
      ROS_INFO("Time elapsed (Intensity_Filter) : %f", t.elapsed());
      t.restart ();
      t_check = 1;
    }
  }*/
  // Test specialized template for sensor_msgs::PointCloud2 point_type

  void
  PointCloudSubCallback (pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc)
  {
    //ROS_INFO("PointCloudSubCallback");
    cob_3d_mapping_filters::IntensityFilter<pcl::PointXYZI> filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
    filter.setInputCloud (pc);
    filter.setFilterLimits (lim_min_, lim_max_);
    //std::cout << "  Filter Limit: "<<filter.getFilterLimit()<< std::endl;
    filter.applyFilter (*cloud_filtered);

    point_cloud_pub_.publish (cloud_filtered);
    if (t_check == 0)
    {
      ROS_INFO("Time elapsed (Intensity_Filter) : %f", t.elapsed());
      t.restart ();
      t_check = 1;
    }
  }

  ros::NodeHandle n_;
  boost::timer t;

protected:
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;

  int lim_min_, lim_max_;
  bool t_check;
};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_filters, IntensityFilter, IntensityFilter, nodelet::Nodelet)

