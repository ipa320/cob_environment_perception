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
 * Date of creation: 02/2012
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
#include <sensor_msgs/image_encodings.h> 

#include <pcl/io/io.h>

// Package includes
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_features/segmentation_nodelet.h"


void
cob_3d_mapping_features::SegmentationNodelet::received_cloud_cb(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  PrecisionStopWatch t;
  t.precisionStart();
  std::cout << "Start .... ";

  one_.setInputCloud(cloud);
  one_.compute(*normals_);
  pcl::copyPointCloud<PointXYZRGB,PointXYZRGB>(*cloud,*colored_);
  pcl::copyPointCloud<PointXYZRGB,PointXYZRGB>(*cloud,*classified_);
  //oce_.setInputCloud(cloud);
  //oce_.compute(*pc_);

  seg_.setInputPoints(cloud);
  seg_.propagateWavefront(cluster_list_);
  seg_.propagateWavefront2ndPass(cluster_list_);
  seg_.getColoredCloud(cluster_list_, colored_);
  //cv::Mat segmented;
  //seg_.getClusterIndices(labels_, clusters_, segmented);

  //cv_bridge::CvImage cv_ptr;
  //cv_ptr.image = segmented;
  //cv_ptr.encoding = sensor_msgs::image_encodings::RGB8;
  //image_pub_.publish(cv_ptr.toImageMsg());
  seg_.analyseClusters(cluster_list_);
  seg_.getColoredCloudByType(cluster_list_, classified_);
  pub_.publish(*colored_);
  classify_pub_.publish(*classified_);

  std::cout << t.precisionStop() << "s\t for segmentation!" << std::endl;
}

void
cob_3d_mapping_features::SegmentationNodelet::onInit()
{
  PCLNodelet::onInit();
  //h_viewer_ = new pcl::visualization::CloudViewer("Nodelet Test!");

  one_.setOutputLabels(labels_);
  one_.setPixelSearchRadius(8,2,2);
  one_.setSkipDistantPointThreshold(8);

  oce_.setInputNormals(normals_);
  oce_.setOutputLabels(labels_);
  oce_.setPixelSearchRadius(8,2,2);
  oce_.setSkipDistantPointThreshold(8);
  oce_.setEdgeClassThreshold(6.0);

  seg_.setInputNormals(normals_);
  seg_.setInputCurvatures(pc_);
  seg_.setOutputLabels(labels_);

  nh_ = getNodeHandle();
  it_ = image_transport::ImageTransport(nh_);
  sub_ = nh_.subscribe<pcl::PointCloud<PointT> >
    ("cloud_in", 1, boost::bind(&cob_3d_mapping_features::SegmentationNodelet::received_cloud_cb, this, _1));
  image_pub_ = it_.advertise("segmentation_image", 1);
  pub_ = nh_.advertise<pcl::PointCloud<PointT> >("segmentation_cloud", 1);
  classify_pub_ = nh_.advertise<pcl::PointCloud<PointT> >("classified_cloud", 1);

}

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_features, SegmentationNodelet, cob_3d_mapping_features::SegmentationNodelet, nodelet::Nodelet);
