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
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
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

#include <sstream>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include "cob_3d_segmentation/simple_segmentation_nodelet.h"
#include "cob_3d_mapping_filters/downsample_filter.h"

void
cob_3d_segmentation::SimpleSegmentationNodelet::onInit()
{
  PCLNodelet::onInit();
  nh_ = getNodeHandle();
  config_server_.reset(new dynamic_reconfigure::Server<segmentation_nodeletConfig>(getPrivateNodeHandle()));
  config_server_->setCallback(boost::bind(&SimpleSegmentationNodelet::configCallback, this, _1, _2));

  one_.setOutputLabels(labels_);
  one_.setPixelSearchRadius(8,2,2);
  one_.setSkipDistantPointThreshold(8);

  seg_.setNormalCloudIn(normals_);
  seg_.setLabelCloudInOut(labels_);
  seg_.setSeedMethod(SEED_RANDOM);

  cc_.setClusterHandler(seg_.clusters());

  sub_points_ = nh_.subscribe<PointCloud>("cloud_in", 1, boost::bind(&SimpleSegmentationNodelet::receivedCloudCallback, this, _1));
  pub_segmented_ = nh_.advertise<PointCloud>("/segmentation/segmented_cloud", 1);
  pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("/segmentation/shape_array",1);
}


void
cob_3d_segmentation::SimpleSegmentationNodelet::configCallback(
  cob_3d_segmentation::segmentation_nodeletConfig& config,
    uint32_t level)
{
  NODELET_INFO("[segmentation]: received new parameters");
  centroid_passthrough_ = config.centroid_passthrough;
  filter_ = config.filter;
  min_cluster_size_ = config.min_cluster_size;
  downsample_ = config.downsample;
}

void
cob_3d_segmentation::SimpleSegmentationNodelet::receivedCloudCallback(PointCloud::ConstPtr cloud)
{
  PrecisionStopWatch t, t2;
  NODELET_INFO("Start with downsampling .... ");
  t2.precisionStart();
  if(downsample_)
  {
    cob_3d_mapping_filters::DownsampleFilter<pcl::PointXYZRGB> down;
    down_->header = cloud->header;
    down.setInputCloud(cloud);
    down.filter(*down_);
    *segmented_ = *down_;
  }
  else
  {
    *segmented_ = *down_ = *cloud;
  }

  NODELET_INFO("Start with segmentation .... ");
  one_.setInputCloud(down_);
  one_.compute(*normals_);

  seg_.setInputCloud(down_);
  t.precisionStart();
  seg_.compute();
  std::cout << "segmentation took " << t.precisionStop() << " s." << std::endl;
  seg_.mapSegmentColor(segmented_);

  pub_segmented_.publish(segmented_);

  // --- start shape array publisher: ---
  cc_.setInputCloud(down_);
  cob_3d_mapping_msgs::ShapeArray sa;
  sa.header = down_->header;
  sa.header.frame_id = down_->header.frame_id.c_str();
  for (ClusterPtr c = seg_.clusters()->begin(); c != seg_.clusters()->end(); ++c)
  {
    if(c->size() < min_cluster_size_) continue;
    Eigen::Vector3f centroid = c->getCentroid();
    if(c->getCentroid()(2) > centroid_passthrough_) continue;
    if(c->size() <= ceil(1.1f * (float)c->border_points.size()))  continue;

    seg_.clusters()->computeClusterComponents(c);
    if(filter_ && !c->is_save_plane) continue;

    sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
    if(!cc_.clusterToShapeMsg(c,sa.shapes.back())) {
      sa.shapes.pop_back();
    }
  }

  pub_shape_array_.publish(sa);
  std::cout << "total nodelet time: " << t2.precisionStop() << std::endl;
}

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SimpleSegmentationNodelet, cob_3d_segmentation::SimpleSegmentationNodelet, nodelet::Nodelet);
