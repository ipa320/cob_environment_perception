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
 *  ROS package name: cob_table_object_cluster
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 07/2013
 *
 * \brief
 * Description:
 *
 * ToDo:
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

// standard includes
//--
//#include <sstream>
//#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

#include <cob_3d_mapping_common/stop_watch.h>

namespace cob_table_object_cluster
{
//####################
//#### nodelet class ####
class TableRegionCropNodelet  : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  // Constructor
  TableRegionCropNodelet()
  : target_frame_id_("/map"),
    hull_(new PointCloud)
  {
    hull_->resize(4);
  }

  // Destructor
  ~TableRegionCropNodelet()
  {
    /// void
  }

  void
  onInit()
  {
    n_ = getNodeHandle();

    ros::NodeHandle pn = getPrivateNodeHandle();
    pn.getParam("target_frame_id", target_frame_id_);
    XmlRpc::XmlRpcValue v;
    pn.getParam("hull_points", v);
    if( v.size() < 12)
    {
      ROS_ERROR("Hull points not set correctly, nodelet will not work");
      return;
    }
    for(int i = 0; i < v.size(); i+=3)
    {
      Point p;
      p.x = (double)(v[i]);
      p.y = (double)(v[i+1]);
      p.z = (double)(v[i+2]);
      hull_->points[i/3] = p;
    }

    pc_sub_ = n_.subscribe<PointCloud>("point_cloud_in", 1, boost::bind(&TableRegionCropNodelet::topicCallback, this, _1));
    pc_pub_ = n_.advertise<PointCloud>("point_cloud_out", 1);

    eppd_.setInputPlanarHull(hull_);
  }

  void
  topicCallback(const PointCloud::ConstPtr& pc_in)
  {
    if(pc_in->header.frame_id != target_frame_id_)
    {
      ROS_ERROR("Frame ID (%s) of incoming point cloud does not match target frame ID (%s), aborting...", pc_in->header.frame_id.c_str(), target_frame_id_.c_str());
      return;
    }

    PointCloud::Ptr pc_out(new PointCloud);
    pcl::PointIndices::Ptr ind_out(new pcl::PointIndices);
    eppd_.setInputCloud(pc_in);
    eppd_.setHeightLimits(0.01, 0.5);
    eppd_.setViewPoint(0,0,5);
    eppd_.segment(*ind_out);
    ei_.setInputCloud(pc_in);
    ei_.setIndices(ind_out);
    ei_.filter(*pc_out);
    pc_pub_.publish(pc_out);
  }


protected:
  ros::NodeHandle n_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
  std::string target_frame_id_;

  PointCloud::Ptr hull_;
  pcl::ExtractPolygonalPrismData<Point> eppd_;
  pcl::ExtractIndices<Point> ei_;
};
}

PLUGINLIB_DECLARE_CLASS(cob_table_object_cluster, TableRegionCropNodelet, cob_table_object_cluster::TableRegionCropNodelet, nodelet::Nodelet);


