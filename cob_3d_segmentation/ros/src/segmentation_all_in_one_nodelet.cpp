/****************************************************************
 *
 * Copyright (c) 2011
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
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
 * ToDo:
 *
 *
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

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/io/io.h>

// Package includes
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_segmentation/segmentation_all_in_one_nodelet.h"



void
cob_3d_segmentation::SegmentationAllInOneNodelet::onInit()
{
  PCLNodelet::onInit();

  one_.setOutputLabels(labels_);
  one_.setPixelSearchRadius(8,2,2);
  one_.setSkipDistantPointThreshold(8);

  seg_.setNormalCloudIn(normals_);
  seg_.setLabelCloudInOut(labels_);
  seg_.setClusterGraphOut(graph_);

  cc_.setClusterHandler(graph_->clusters());
  cc_.setNormalCloudInOut(normals_);
  cc_.setLabelCloudIn(labels_);

  nh_ = getNodeHandle();
  sub_points_ = nh_.subscribe<PointCloud>
    ("cloud_in", 1, boost::bind(&cob_3d_segmentation::SegmentationAllInOneNodelet::received_cloud_cb, this, _1));
  pub_segmented_ = nh_.advertise<PointCloud>("segmentation_cloud", 1);
  pub_classified_ = nh_.advertise<PointCloud>("classified_cloud", 1);
  std::cout << "Loaded" << std::endl;

}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::received_cloud_cb(PointCloud::ConstPtr cloud)
{
  PrecisionStopWatch t;
  t.precisionStart();
  NODELET_INFO("Start .... ");

  one_.setInputCloud(cloud);
  one_.compute(*normals_);
  *classified_ = *segmented_ = *cloud;

  seg_.setInputCloud(cloud);
  seg_.performInitialSegmentation();
  seg_.refineSegmentation();
  graph_->clusters()->mapClusterColor(segmented_);
  cc_.setPointCloudIn(cloud);
  cc_.classify();
  graph_->clusters()->mapTypeColor(classified_);
  pub_segmented_.publish(segmented_);
  pub_classified_.publish(classified_);
  
  NODELET_INFO("Done .... ");
}

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SegmentationAllInOneNodelet, cob_3d_segmentation::SegmentationAllInOneNodelet, nodelet::Nodelet);
