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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: cob_env_model
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
  //PCLNodelet::onInit();
  nh_ = getNodeHandle();
  config_server_.reset(new dynamic_reconfigure::Server<segmentation_nodeletConfig>(getPrivateNodeHandle()));
  config_server_->setCallback(boost::bind(&SimpleSegmentationNodelet::configCallback, this, _1, _2));

  one_.setOutputLabels(labels_);
  one_.setPixelSearchRadius(8,2,2);
  one_.setSkipDistantPointThreshold(8);

  seg_.setNormalCloudIn(normals_);
  seg_.setLabelCloudInOut(labels_);
  seg_.setSeedMethod(SEED_RANDOM);

  sub_points_ = nh_.subscribe<PointCloud>("point_cloud", 1, boost::bind(&SimpleSegmentationNodelet::topicCallback, this, _1));
  pub_segmented_ = nh_.advertise<PointCloud>("segmented_cloud", 1);
  pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("shape_array",1);
  as_ = new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerAction>(nh_, "segmentation/trigger", boost::bind(&SimpleSegmentationNodelet::actionCallback, this, _1), false);
  as_->start();
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
  colorize_ = config.colorize;
  enable_action_mode_ = config.enable_action_mode;
  if(!is_running_ && !enable_action_mode_)
  { // Start immediately
    ROS_INFO("Starting segmentation...");
    sub_points_ = nh_.subscribe("point_cloud", 1, &SimpleSegmentationNodelet::topicCallback, this);
    is_running_ = true;
  }
}

void
cob_3d_segmentation::SimpleSegmentationNodelet::actionCallback(const cob_3d_mapping_msgs::TriggerGoalConstPtr& goal)
{
  //boost::lock_guard<boost::mutex> guard(mutex_);
  cob_3d_mapping_msgs::TriggerResult result;
  if(goal->start && !is_running_)
  {
    ROS_INFO("Starting segmentation...");
    sub_points_ = nh_.subscribe("point_cloud", 1, &SimpleSegmentationNodelet::topicCallback, this);
    is_running_ = true;
  }
  else if(!goal->start && is_running_)
  {
    ROS_INFO("Stopping segmentation...");
    sub_points_.shutdown();
    is_running_ = false;
  }

  as_->setSucceeded(result);
}

void
cob_3d_segmentation::SimpleSegmentationNodelet::topicCallback(PointCloud::ConstPtr cloud)
{
  boost::lock_guard<boost::mutex> guard(mutex_);
  PrecisionStopWatch t;
  NODELET_INFO("Received PointCloud. Start downsampling .... ");
  t.precisionStart();
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
  std::cout << "Downsampling took " << t.precisionStop() << " s." << std::endl;
  computeAndPublish();
}

void
cob_3d_segmentation::SimpleSegmentationNodelet::computeAndPublish()
{
  PrecisionStopWatch t, t2;
  t2.precisionStart();
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
  PointCloud::Ptr hull(new PointCloud);
  cob_3d_mapping_msgs::ShapeArray sa;
  sa.header = down_->header;
  sa.header.frame_id = down_->header.frame_id.c_str();
  unsigned int id = 0;
  for (ClusterPtr c = seg_.clusters()->begin(); c != seg_.clusters()->end(); ++c)
  {
    if(c->size() < min_cluster_size_) {/*std::cout << "da1" << std::endl;*/continue;}
    Eigen::Vector3f centroid = c->getCentroid();
    if(centroid(2) > centroid_passthrough_) {/*std::cout << "da2" << std::endl;*/continue;}
    if(c->size() <= ceil(1.1f * (float)c->border_points.size()))  {/*std::cout << "da3" << std::endl;*/continue;}

    seg_.clusters()->computeClusterComponents(c);
    if(filter_ && !c->is_save_plane) {/*std::cout << "da4" << std::endl;*/continue;}


    PolygonContours<PolygonPoint> poly;
    std::cout << c->border_points.size() << std::endl;
    pe_.outline(down_->width, down_->height, c->border_points, poly);
    if(!poly.polys_.size()) {/*std::cout << "da5" << std::endl;*/continue;} // continue, if no contours were found
    int max_idx=0, max_size=0;
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      if ((int)poly.polys_[i].size() > max_size) { max_idx = i; max_size = poly.polys_[i].size(); }
    }

    sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
    cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
    s->id = id++;
    s->points.resize(poly.polys_.size());
    s->header.frame_id = down_->header.frame_id.c_str();
    /*Eigen::Vector3f color = c->computeDominantColorVector().cast<float>();
    float tmp_inv = 1.0f / 255.0f;*/
    s->color.r = 0;//color(0) * tmp_inv;
    s->color.g = 0;//color(1) * tmp_inv;
    s->color.b = 0.0f;//color(2) * tmp_inv;
    s->color.a = 1.0f;

    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      if (i == max_idx)
      {
        s->holes.push_back(false);
        std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin();
        for ( ; it != poly.polys_[i].end(); ++it) {
          hull->push_back( down_->points[ it->x + it->y * down_->width ] );
        }
      }
      else
      {
        s->holes.push_back(true);
        std::vector<PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin();
        for ( ; it != poly.polys_[i].rend(); ++it) {
          hull->push_back( down_->points[ it->x + it->y * down_->width ] );
        }
      }
      hull->height = 1;
      hull->width = hull->size();
      pcl::toROSMsg(*hull, s->points[i]);
      hull->clear();
    }

    s->centroid.x = centroid[0];
    s->centroid.y = centroid[1];
    s->centroid.z = centroid[2];
    s->type = cob_3d_mapping_msgs::Shape::POLYGON;
    s->params.resize(4);
    s->params[0] = c->pca_point_comp3(0);
    s->params[1] = c->pca_point_comp3(1);
    s->params[2] = c->pca_point_comp3(2);
    if(colorize_)
    {
      s->params[3] = fabs(centroid.dot(c->pca_point_comp3)); // d
      Eigen::Vector3f color = c->computeDominantColorVector().cast<float>();
      float temp_inv = 1.0f/255.0f;
      s->color.r = color(0) * temp_inv;
      s->color.g = color(1) * temp_inv;
      s->color.b = color(2) * temp_inv;
      s->color.a = 1.0f;
    }
  }

  pub_shape_array_.publish(sa);
  std::cout << "total nodelet time: " << t2.precisionStop() << std::endl;
}

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SimpleSegmentationNodelet, cob_3d_segmentation::SimpleSegmentationNodelet, nodelet::Nodelet);
