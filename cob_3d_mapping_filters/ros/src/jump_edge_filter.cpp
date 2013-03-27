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
 * ROS package name: cob_3d_mapping_filters
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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

//pludinlib includes
#include <pluginlib/class_list_macros.h>

// pcl includes
#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/pcd_io.h>

// cob_3d_mapping_filters includes
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_filters/jump_edge_filter.h>
#include <cob_3d_mapping_filters/impl/jump_edge_filter.hpp>

//######################
//#### nodelet class####
class JumpEdgeFilter : public pcl_ros::PCLNodelet
{
  public:
  // Constructor
  JumpEdgeFilter()
   :t_check(0)
  {}

  // Destructor
  ~ JumpEdgeFilter()
  {
    /// void
  }

  void onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    point_cloud_sub_ = n_.subscribe("point_cloud2", 1, & JumpEdgeFilter::PointCloudSubCallback, this);
    point_cloud_pub_= n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_filtered",1);

    n_.param("/jump_edge_filter_nodelet/upper_angle_deg", upper_angle_, 170.0);
    //std::cout << "upper_angle_deg: " << upper_angle_<< std::endl;
  }

/*  void PointCloudSubCallback(const pcl::PointCloud<CPCPoint>::Ptr pc)
  {
    //ROS_INFO("PointCloudSubCallback");
    cob_3d_mapping_filters::JumpEdgeFilter<CPCPoint> filter;
    pcl::PointCloud<CPCPoint>::Ptr cloud_filtered(new pcl::PointCloud<CPCPoint> ());

    filter.setInputCloud(pc);
    filter.setUpperAngle(upper_angle_);
    //std::cout << "  Upper angle threshold in degrees : "<<filter.getUpperAngle()  << std::endl;
    filter.applyFilter(*cloud_filtered);
    point_cloud_pub_.publish(cloud_filtered);
    if(t_check==0)
    {
      ROS_INFO("Time elapsed (JumpEdgeFilter) : %f", t.elapsed());
      t.restart();
      t_check=1;
    }
  }*/

  // Test specialized template for sensor_msgs::PointCloud2 point_type

  void
  PointCloudSubCallback (pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc)
  {
    //ROS_INFO("PointCloudSubCallback");
    cob_3d_mapping_filters::JumpEdgeFilter<pcl::PointXYZI> filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());

    filter.setInputCloud (pc);
    filter.setUpperAngle(upper_angle_);
    filter.filter (*cloud_filtered);
    point_cloud_pub_.publish (cloud_filtered);
    if (t_check == 0)
    {
      ROS_INFO("Time elapsed (JumpEdgeFilter) : %f", t.elapsed());
      t.restart ();
      t_check = 1;
    }
  }

  ros::NodeHandle n_;
  boost::timer t;

  protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;

    double upper_angle_;
    bool t_check;
  };

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_filters,  JumpEdgeFilter,  JumpEdgeFilter, nodelet::Nodelet)


