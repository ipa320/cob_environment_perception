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
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_segmentation/simple_segmentation_nodelet.h"
#include "cob_3d_mapping_filters/downsample_filter.h"

void
cob_3d_segmentation::SimpleSegmentationNodelet::onInit()
{
  //PCLNodelet::onInit();
  nh_ = getNodeHandle();
  config_server_.reset(
    new dynamic_reconfigure::Server<segmentation_nodeletConfig>(
      getPrivateNodeHandle()));
  config_server_->setCallback(
    boost::bind(&SimpleSegmentationNodelet::configCallback, this, _1, _2));

  one_.setOutputLabels(labels_);
  one_.setPixelSearchRadius(8,2,2);
  one_.setSkipDistantPointThreshold(8);

  seg_.setNormalCloudIn(normals_);
  seg_.setLabelCloudInOut(labels_);
  seg_.setSeedMethod(SEED_RANDOM);

  pub_segmented_ = nh_.advertise<PointCloud>("segmented_cloud", 1);
  pub_classified_ = nh_.advertise<PointCloud>("classified_cloud", 1);
  pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("shape_array",1);
  as_ = new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerAction>(
    nh_, "segmentation/trigger",
    boost::bind(&SimpleSegmentationNodelet::actionCallback, this, _1), false);
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
  skip_level_ = config.skip_level;
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
cob_3d_segmentation::SimpleSegmentationNodelet::actionCallback(
  const cob_3d_mapping_msgs::TriggerGoalConstPtr& goal)
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
cob_3d_segmentation::SimpleSegmentationNodelet::topicCallback(
  const PointCloud::ConstPtr& cloud)
{
  //boost::lock_guard<boost::mutex> guard(mutex_);
  PrecisionStopWatch t;
  NODELET_INFO("Received PointCloud. Start downsampling .... ");

  //t.precisionStart();
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
  //std::cout << "Downsampling took " << t.precisionStop() << " s." << std::endl;
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
  //t.precisionStart();
  seg_.compute();
  NODELET_INFO("Segmentation took %f s.", t.precisionStop());

  if(pub_segmented_.getNumSubscribers()>0) {
    seg_.mapSegmentColor(segmented_);
    pub_segmented_.publish(segmented_);
  }

  // --- start shape array publisher: ---
  PointCloud::Ptr hull(new PointCloud);
  cob_3d_mapping_msgs::ShapeArray sa;
  pcl_conversions::fromPCL(down_->header, sa.header);
  //sa.header = down_->header;
  //sa.header.frame_id = down_->header.frame_id.c_str();
  //unsigned int id = 0;
  for (ClusterPtr c = seg_.clusters()->begin(); c != seg_.clusters()->end(); ++c)
  {
    if(c->size() < min_cluster_size_) {continue;} //segment too small
    if(c->getCentroid()(2) > centroid_passthrough_) {continue;} //segment too far away
    if(c->border_points.size() > ceil(0.9f * (float)c->size()))  {continue;} //ratio of border points too high => long and thin segment

    seg_.clusters()->computeClusterComponents(c);
    if(filter_ && !c->is_planar_) {continue;} //segment is not a plane

    cob_3d_mapping::Polygon::Ptr p;
    //std::cout << "cluster id " << c->id() << std::endl;
    conv_.setColor(colorize_);
    conv_.setInputCloud(down_);
    if(!conv_.clusterToPolygon(c, p, skip_level_)) continue;

    sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
    cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
    pcl_conversions::fromPCL(down_->header, s->header);
    //s->header = down_->header;

    //computeTexture(c, p->pose_, p->id_);
    cob_3d_mapping::toROSMsg(*p, *s);
    //id++;
  }

  pub_shape_array_.publish(sa);
  publishClassifiedCloud();
  std::cout << "total nodelet time: " << t2.precisionStop() << std::endl;
}


void
cob_3d_segmentation::SimpleSegmentationNodelet::publishClassifiedCloud()
{
  seg_.clusters()->mapTypeColor(down_);
  seg_.clusters()->mapClusterBorders(down_);
//  for (ClusterPtr c = seg_.clusters()->begin(); c != seg_.clusters()->end(); ++c)
//  {
//    Eigen::Vector3f centroid = c->getCentroid();
//    //if(centroid(2) > centroid_passthrough_) {continue;}
//    //seg_.clusters()->computeClusterComponents(c);
//    uint32_t color = LBL_BORDER;
//    // not a plane
//    //if(/*c->size() < min_cluster_size_ || c->size() <= ceil(1.1f * (float)c->border_points.size()) ||*/ (filter_ && !c->is_save_plane))
//    /*{
//      color = LBL_SPH;
//    }
//    else
//    {
//      color = LBL_PLANE;
//    }*/
//    if(c->id() == I_NAN) color = LBL_BORDER;
//    /*for(unsigned int i = 0; i < c->indices_.size(); i++)
//    {
//      down_->points[c->indices_[i]].rgb = color;
//    }*/
//    PolygonContours<PolygonPoint> poly;
//    pe_.outline(down_->width, down_->height, c->border_points, poly);
//    for (int i = 0; i < (int)poly.polys_.size(); ++i)
//    {
//      std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin();
//      for ( ; it != poly.polys_[i].end(); ++it)
//        down_->points[ it->x + it->y * down_->width ].rgb = LBL_BORDER;;
//    }
//    for(std::vector<PolygonPoint>::iterator bp = c->border_points.begin(); bp != c->border_points.end(); ++bp)
//    {
//      down_->points[PolygonPoint::getInd(bp->x,bp->y)].rgb = LBL_BORDER;
//    }
//    /*for(unsigned int i = 0; i < c->border_points.size(); i++)
//    {
//      down_->points[PolygonPoint::getInd(c->border_points[i].x,c->border_points[i].y)].r = 255;
//      down_->points[PolygonPoint::getInd(c->border_points[i].x,c->border_points[i].y)].g = 0;
//      down_->points[PolygonPoint::getInd(c->border_points[i].x,c->border_points[i].y)].b = 0;
//    }*/
//  }
  pub_classified_.publish(down_);
}

void
cob_3d_segmentation::SimpleSegmentationNodelet::computeTexture(
  ClusterPtr &c, Eigen::Affine3f &trf, unsigned int id)
{
  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud(down_);
  pcl::IndicesPtr ind_ptr(new std::vector<int>);
  for(unsigned int i=0; i<c->indices_.size(); i++)
    ind_ptr->push_back(c->indices_[i]);
  ei.setIndices(ind_ptr);
  PointCloud::Ptr segment(new PointCloud);
  ei.filter(*segment);
  PointCloud::Ptr segment_tr(new PointCloud);
  pcl::transformPointCloud(*segment, *segment_tr, trf.inverse());
  std::stringstream ss;
  ss << "/tmp/seg_" << id << ".pcd";
  pcl::io::savePCDFileBinary(ss.str(), *segment_tr);
  double mToPx = 500;
  int min_u = std::numeric_limits<int>::max(), max_u = std::numeric_limits<int>::min(), min_v = std::numeric_limits<int>::max(), max_v  = std::numeric_limits<int>::min();
  for (unsigned int i=0; i<segment_tr->size(); i++)
  {
    int u = round(segment_tr->points[i].x * mToPx);
    int v = round(segment_tr->points[i].y * mToPx);
    if(u < min_u) min_u = u;
    if(u > max_u) max_u = u;
    if(v < min_v) min_v = v;
    if(v > max_v) max_v = v;
    //std::cout << segment_tr->points[i].x << "," << segment_tr->points[i].y << ":" << u << "," << v << std::endl;
  }
  //TODO: save min/max u v for transforming back
  //std::cout << "u(min, max): " << min_u << "," << max_u << " v(min,max): " << min_v << "," << max_v << std::endl;
  cv::Mat img(abs(max_u - min_u) +1 , abs(max_v - min_v) + 1, CV_8UC3, cv::Scalar(-1));
  //std::cout << "im size: " << img.rows << "," << img.cols << std::endl;
  for (unsigned int i=0; i<segment_tr->size(); i++)
  {
    //TODO: handle case that both u_min and u-Max are negative (same for v)
    int u = round(segment_tr->points[i].x * mToPx) - min_u;
    int v = round(segment_tr->points[i].y * mToPx) - min_v;
    //std::cout << "u,v" << u << "," << v << std::endl;
    img.at<cv::Vec3b>(u,v)[2] = segment_tr->points[i].r;
    img.at<cv::Vec3b>(u,v)[1] = segment_tr->points[i].g;
    img.at<cv::Vec3b>(u,v)[0] = segment_tr->points[i].b;
  }
  std::stringstream ss1;
  ss1 << "/tmp/seg_" << id << ".png";
  cv::imwrite(ss1.str(), img);

  cv::Mat img_dil(abs(max_u - min_u) +1 , abs(max_v - min_v) + 1, CV_8UC3);
  cv::dilate(img, img_dil, cv::Mat(), cv::Point(-1,-1), 2);
  std::stringstream ss2;
  ss2 << "/tmp/seg_" << id << "_dil.png";
  cv::imwrite(ss2.str(), img_dil);

  PointCloud::Ptr segm(new PointCloud);
  //pcl::copyPointCloud(*down_, copy);
  pcl::ExtractIndices<pcl::PointXYZRGB> ei2;
  ei2.setInputCloud(down_);
  ei2.setIndices(ind_ptr);
  ei2.setKeepOrganized(true);
  ei2.filter(*segm);

  pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;
  pcl::PolygonMesh pcl_mesh;
  ofm.setInputCloud(segm);
  //ofm.setIndices(ind_ptr);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
  ofm.reconstruct(pcl_mesh);
  /*try
  {
    std::stringstream ss3;
    ss3 << "/tmp/mesh_" << id << ".ply";
    //pcl::io::saveVTKFile(ss3.str(), pcl_mesh);
    pcl::io::savePLYFile(ss3.str(), pcl_mesh);
  }
  catch( std::exception& x )
  {
    std::cerr << x.what() << std::endl;
    return;
  }*/

}

PLUGINLIB_EXPORT_CLASS(cob_3d_segmentation::SimpleSegmentationNodelet, nodelet::Nodelet);
