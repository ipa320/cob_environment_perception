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
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// pcl includes
#define PCL_NO_PRECOMPILE
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/impl/pcl_base.hpp>

// cob_3d_mapping_filters includes
#include <cob_3d_mapping_common/point_types.h>

//######################
//#### nodelet class####
class AmplitudeFilter : public nodelet::Nodelet
{
public:
  typedef PointXYZA PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  // Constructor
  AmplitudeFilter ()
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
    n_ = getPrivateNodeHandle ();

    point_cloud_sub_ = n_.subscribe ("point_cloud_in", 1, &AmplitudeFilter::pointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<PointCloud> ("point_cloud_out", 1);

    double lim_min, lim_max;
    n_.param ("amplitude_min_threshold", lim_min, 2000.);
    n_.param ("amplitude_max_threshold", lim_max, 60000.);
    bool keep_organized;
    n_.param ("keep_organized", keep_organized, false);
    
    std::string field="amplitude";
    n_.getParam("field", field);
    
    filter_.setFilterLimits (lim_min, lim_max);
    filter_.setFilterFieldName (field);
    filter_.setKeepOrganized(keep_organized);
  }

  void
  pointCloudSubCallback (PointCloud::ConstPtr pc)
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

  pcl::PassThrough<PointT> filter_;
};

PLUGINLIB_EXPORT_CLASS(AmplitudeFilter, nodelet::Nodelet);

