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
 *  Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
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
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/io/pcd_io.h>

// cob_3d_mapping_filters includes
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_filters/speckle_filter.h>
#include <cob_3d_mapping_filters/impl/speckle_filter.hpp>

//######################
//#### nodelet class####
class SpeckleFilter : public nodelet::Nodelet
{
public:
  // Constructor
  SpeckleFilter () :
    t_check (0)
  {
    //
  }

  // Destructor
  ~ SpeckleFilter ()
  {
    /// void
  }

  void
  onInit ()
  {
    //PCLNodelet::onInit ();
    n_ = getNodeHandle ();

    point_cloud_sub_ = n_.subscribe ("point_cloud2", 1, &SpeckleFilter::PointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2> ("point_cloud2_filtered", 1);

    n_.param ("/speckle_filter_nodelet/speckle_size", speckle_size_, 50);
    //std::cout << "speckle_size: " << speckle_size_<< std::endl;
    n_.param ("/speckle_filter_nodelet/speckle_range", speckle_range_, 0.1);
    std::cout << "speckle_range: " << speckle_range_<< std::endl;
  }

  void
   PointCloudSubCallback (const pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
   {
     //ROS_INFO("PointCloudSubCallback");
     cob_3d_mapping_filters::SpeckleFilter<pcl::PointXYZ> filter;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
     filter.setInputCloud (pc);
     filter.setFilterParam(speckle_size_,speckle_range_);
     //std::cout<< " SR : "<<filter.getSpeckleRange()<<std::endl;
     //std::cout<< " SS : "<<filter.getSpeckleSize()<<std::endl;
     filter.filter (*cloud_filtered);
     point_cloud_pub_.publish (cloud_filtered);
     if (t_check == 0)
     {
       ROS_INFO("Time elapsed (SpeckleFilter) : %f", t.elapsed());
       t.restart ();
       t_check = 1;
     }
   }

  ros::NodeHandle n_;
  boost::timer t;
  bool t_check;

protected:
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;

  int speckle_size_;
  double speckle_range_;
};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_filters, SpeckleFilter, SpeckleFilter, nodelet::Nodelet)

