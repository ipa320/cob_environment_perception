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
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>

// pcl includes
#include <pcl/point_types.h>

// cob_3d_mapping_filters includes
//#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_filters/jump_edge_filter.h>
#include <cob_3d_mapping_filters/impl/jump_edge_filter.hpp>

//######################
//#### nodelet class####
class JumpEdgeFilter : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  // Constructor
  JumpEdgeFilter ()
  {
  }

  // Destructor
  ~ JumpEdgeFilter ()
  {
    /// void
  }

  void
  onInit ()
  {
    n_ = getPrivateNodeHandle ();

    point_cloud_sub_ = n_.subscribe ("point_cloud_in", 1, &JumpEdgeFilter::pointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<PointCloud> ("point_cloud_out", 1);

    double angle;
    n_.param ("upper_angle_deg", angle, 170.0);
    bool keep_organized;
    n_.param ("keep_organized", keep_organized, false);
    filter_.setAngleThreshold (angle);
    filter_.setKeepOrganized(keep_organized);
  }

  void
  pointCloudSubCallback (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc)
  {
    PointCloud cloud_filtered;

    filter_.setInputCloud (pc);
    filter_.filter (cloud_filtered);
    point_cloud_pub_.publish (cloud_filtered);
  }

protected:
  ros::NodeHandle n_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;

  cob_3d_mapping_filters::JumpEdgeFilter<PointT> filter_;
};

PLUGINLIB_EXPORT_CLASS(JumpEdgeFilter, nodelet::Nodelet);

