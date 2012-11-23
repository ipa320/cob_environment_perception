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
}

void
cob_3d_segmentation::SimpleSegmentationNodelet::receivedCloudCallback(PointCloud::ConstPtr cloud)
{
  PrecisionStopWatch t, t2;
  NODELET_INFO("Start with segmentation .... ");
  t2.precisionStart();
  one_.setInputCloud(cloud);
  one_.compute(*normals_);

  *segmented_ = *cloud;
  seg_.setInputCloud(cloud);
  seg_.createSeedPoints();
  t.precisionStart();
  seg_.compute();
  std::cout << "segmentation took " << t.precisionStop() << " s." << std::endl;
  seg_.mapSegmentColor(segmented_);

  pub_segmented_.publish(segmented_);

  PointCloud::Ptr hull(new PointCloud);
  cob_3d_mapping_msgs::ShapeArray sa;
  sa.header = cloud->header;
  sa.header.frame_id = cloud->header.frame_id.c_str();
  for (ClusterPtr c = seg_.clusters()->begin(); c != seg_.clusters()->end(); ++c)
  {
    if(c->size() < 20) continue;
    Eigen::Vector3f centroid = c->getCentroid();
    if(centroid(2) > centroid_passthrough_) continue;
    if(c->size() <= ceil(1.1f * (float)c->border_points.size())) { /*std::cout << "Size: "<< c->size() << std::endl;*/ continue; }

    PolygonContours<PolygonPoint> poly;
    pe_.outline(cloud->width, cloud->height, c->border_points, poly);
    if(!poly.polys_.size()) continue; // continue, if no contours were found
    int max_idx=0, max_size=0;
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      if ((int)poly.polys_[i].size() > max_size) { max_idx = i; max_size = poly.polys_[i].size(); }
    }

    sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
    cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
    s->id = 0;
    s->points.resize(poly.polys_.size());
    s->header.frame_id = cloud->header.frame_id.c_str();
    Eigen::Vector3f color = c->computeDominantColorVector().cast<float>();
    float tmp_inv = 1.0f / 255.0f;
    s->color.r = color(0) * tmp_inv;
    s->color.g = color(1) * tmp_inv;
    s->color.b = color(2) * tmp_inv;

    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      if (i == max_idx)
      {
        s->holes.push_back(false);
        std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin();
        for ( ; it != poly.polys_[i].end(); ++it)
          hull->push_back( cloud->points[PolygonPoint::getInd(it->x, it->y)] );
      }
      else
      {
        s->holes.push_back(true);
        std::vector<PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin();
        for ( ; it != poly.polys_[i].rend(); ++it)
          hull->push_back( cloud->points[PolygonPoint::getInd(it->x, it->y)] );
      }
      hull->height = 1;
      hull->width = hull->size();
      pcl::toROSMsg(*hull, s->points[i]);
      hull->clear();
    }
    seg_.clusters()->computeClusterComponents(c);

    s->centroid.x = centroid[0];
    s->centroid.y = centroid[1];
    s->centroid.z = centroid[2];
    s->type = cob_3d_mapping_msgs::Shape::POLYGON;
    s->params.resize(4);
    s->params[0] = c->pca_point_comp3(0);
    s->params[1] = c->pca_point_comp3(1);
    s->params[2] = c->pca_point_comp3(2);
    s->params[3] = fabs(centroid.dot(c->pca_point_comp3)); // d
  }

  pub_shape_array_.publish(sa);
  std::cout << "total nodelet time: " << t2.precisionStop() << std::endl;
}

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SimpleSegmentationNodelet, cob_3d_segmentation::SimpleSegmentationNodelet, nodelet::Nodelet);
