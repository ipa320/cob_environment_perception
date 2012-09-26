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

#include <sstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <boost/make_shared.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/for_each_type.h>

// Package includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_common/cylinder.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include "cob_3d_segmentation/segmentation_all_in_one_nodelet.h"


void
cob_3d_segmentation::SegmentationAllInOneNodelet::onInit()
{
  PCLNodelet::onInit();
  nh_ = getNodeHandle();

  config_server_.reset(new dynamic_reconfigure::Server<segmentation_nodeletConfig>(getPrivateNodeHandle()));
  config_server_->setCallback(boost::bind(&SegmentationAllInOneNodelet::configCallback, this, _1, _2));

  one_.setOutputLabels(labels_);
  one_.setPixelSearchRadius(8,2,2);
  one_.setSkipDistantPointThreshold(8);

  seg_.setNormalCloudIn(normals_);
  seg_.setLabelCloudInOut(labels_);
  seg_.setClusterGraphOut(graph_);

  cc_.setClusterHandler(graph_->clusters());
  cc_.setNormalCloudInOut(normals_);
  cc_.setLabelCloudIn(labels_);


  sub_points_ = nh_.subscribe<PointCloud>("cloud_in", 1, boost::bind(&SegmentationAllInOneNodelet::receivedCloudCallback, this, _1));
  pub_segmented_ = nh_.advertise<PointCloud>("/segmentation/segmented_cloud", 1);
  pub_classified_ = nh_.advertise<PointCloud>("/segmentation/classified_cloud", 1);
  pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("/segmentation/shape_array",1);
  pub_chull_ = nh_.advertise<PointCloud>("/segmentation/concave_hull", 1);
  pub_chull_dense_ = nh_.advertise<PointCloud>("/segmentation/concave_hull_dense", 1);
  std::cout << "Loaded segmentation nodelet" << std::endl;
}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::configCallback(
    cob_3d_segmentation::segmentation_nodeletConfig& config,
    uint32_t level)
{
  NODELET_INFO("[segmentation]: received new parameters");
  centroid_passthrough_ = config.centroid_passthrough;
}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::receivedCloudCallback(PointCloud::ConstPtr cloud)
{
  PrecisionStopWatch t;
  t.precisionStart();
  NODELET_INFO("Start with segmentation .... ");
  one_.setInputCloud(cloud);
  one_.compute(*normals_);

  *classified_ = *segmented_ = *cloud;

  seg_.setInputCloud(cloud);
  seg_.performInitialSegmentation();
  seg_.refineSegmentation();
  std::map<int,int> objects;
  //seg_.getPotentialObjects(objects, 500);
  std::cout << "Found " << objects.size() << " potentail objects" << std::endl;
  NODELET_INFO("Done with segmentation .... ");
  graph_->clusters()->mapClusterColor(segmented_);

  cc_.setPointCloudIn(cloud);
  cc_.classify();
  graph_->clusters()->mapTypeColor(classified_);
  graph_->clusters()->mapClusterBorders(classified_);
  NODELET_INFO("publish first cloud .... ");
  pub_segmented_.publish(segmented_);
  NODELET_INFO("publish second cloud .... ");
  pub_classified_.publish(classified_);
  NODELET_INFO("publish shape array .... ");
  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bp (new pcl::PointCloud<pcl::PointXYZRGB>);
   *bp = *cloud;
  graph_->clusters()->mapClusterBorders(bp);
  std::stringstream ss;
  ss << "/share/goa-sf/pcd_data/bags/pcd_borders/borders_"<<cloud->header.stamp<<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *bp);
   */
  publishShapeArray(graph_->clusters(), objects, cloud);

  NODELET_INFO("Done with publishing .... ");

}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::publishShapeArray(
  ST::CH::Ptr cluster_handler, std::map<int,int>& objs, PointCloud::ConstPtr cloud)
{
  cob_3d_mapping_msgs::ShapeArray sa;
  sa.header = cloud->header;
  sa.header.frame_id = cloud->header.frame_id.c_str();
  std::cout<<"[SN]-->CLOUD FRAME"<<cloud->header.frame_id.c_str()<<"\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud_dense(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (ST::CH::ClusterPtr c = cluster_handler->begin(); c != cluster_handler->end(); ++c)
  {
    // compute hull:
    if (c->getCentroid()[2] > centroid_passthrough_) continue;
    if (c->type != I_PLANE && c->type != I_CYL) continue;
    if (c->size() <= ceil(1.1f * static_cast<float>(c->border_points.size())))
    {
      std::cout <<"[ " << c->size() <<" | "<< c->border_points.size() << " ]" << std::endl;
      continue;
    }

    for (size_t i = 0; i < c->border_points.size(); ++i)
      hull_cloud_dense->push_back((*segmented_)[c->border_points[i].y * segmented_->width + c->border_points[i].x]);

    PolygonContours<PolygonPoint> poly;
    std::cout << "Get outline for " << c->size() << " Points with "<< c->border_points.size() << " border points" << std::endl;
    pe_.outline(cloud->width, cloud->height, c->border_points, poly);
    if (!poly.polys_.size()) continue; // continue, if no contours were found

    int max_idx=0, max_size=0;
    std::cout << "Polys: ";
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      std::cout << poly.polys_[i].size() << " ";
      if ((int)poly.polys_[i].size() > max_size) { max_idx = i; max_size = poly.polys_[i].size(); }
    }
    std::cout << std::endl;

    sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
    cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
    if (objs.find(c->id()) != objs.end()) s->id = objs[c->id()] + 1;
    else s->id = 0;
    s->points.resize(poly.polys_.size());
    s->header.frame_id = cloud->header.frame_id.c_str();
    Eigen::Vector3f color = c->computeDominantColorVector().cast<float>();
    float temp_inv = 1.0f/255.0f;
    s->color.r = color(0) * temp_inv;
    s->color.g = color(1) * temp_inv;
    s->color.b = color(2) * temp_inv;
    s->color.a = 1.0f;
    Eigen::Vector3f centroid = c->getCentroid();
    //    Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - c->pca_point_comp3 * c->pca_point_comp3.transpose(); // projection
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      pcl::PointXYZRGB p;
      if (i == max_idx)
      {
        s->holes.push_back(false);
        std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin();
        for ( ; it != poly.polys_[i].end(); ++it)
        {
          p.getVector3fMap() = cloud->points[PolygonPoint::getInd(it->x, it->y)].getVector3fMap();
          hull_cloud->points.push_back(p);
          hull->points.push_back(p);
        }
      }
      else
      {
        s->holes.push_back(true);
        std::vector<PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin();
        for ( ; it != poly.polys_[i].rend(); ++it)
        {
          p.getVector3fMap() = cloud->points[PolygonPoint::getInd(it->x, it->y)].getVector3fMap();
          hull_cloud->points.push_back(p);
          hull->points.push_back(p);
        }
      }
      hull->height = 1;
      hull->width = hull->points.size();
      pcl::toROSMsg(*hull, s->points[i]);
      hull->clear();
    }
    s->centroid.x = centroid[0];
    s->centroid.y = centroid[1];
    s->centroid.z = centroid[2];

    // Set type specific parameters:
    switch(c->type)
    {
    case I_PLANE:
    {
      //std::cout << "Plane: " << c->size() << ", " << c->border_points.size() << std::endl;
      s->type = cob_3d_mapping_msgs::Shape::POLYGON;

      s->params.resize(4);
      Eigen::Vector3f orientation =  c->pca_point_comp3;
      s->params[0] = orientation(0); // n_x
      s->params[1] = orientation(1); // n_y
      s->params[2] = orientation(2); // n_z
      s->params[3] = fabs(centroid.dot(orientation)); // d
      break;
    }
    case I_CYL:
    {
      std::cout<<"CLYINDER is published\n";
      s->type = cob_3d_mapping_msgs::Shape::CYLINDER;
      s->params.resize(10);

      cob_3d_mapping::CylinderPtr  cyl = cob_3d_mapping::CylinderPtr(new cob_3d_mapping::Cylinder());
      Eigen::Vector3f centroid3f  = c->getCentroid();
      cyl->centroid << centroid3f[0] , centroid3f[1] , centroid3f[2] , 0;

      cyl->sym_axis =  c->pca_inter_comp1;
      std::cout<<"sym axis\n"<<cyl->sym_axis<<"\n";
      std::cout<<"centroid\n"<<cyl->centroid<<"\n";
      cyl->ParamsFromCloud(cloud,c->indices_);


      //write parameters to msg - after transformation to target frame
      s->params[0] = cyl->sym_axis[0];
      s->params[1] = cyl->sym_axis[1];
      s->params[2] = cyl->sym_axis[2];

      s->params[3] = cyl->normal[0];
      s->params[4] = cyl->normal[1];
      s->params[5] = cyl->normal[2];

      s->params[6] = cyl->origin_[0];
      s->params[7] = cyl->origin_[1];
      s->params[8] = cyl->origin_[2];

      s->params[9]= cyl->r_;

      break;
    }
    default:
    {
      break;
    }
    }

    /*for (int i = 0; i < sa.shapes.back().points.size(); ++i)
      {
      std::cout << "Size: " << sa.shapes.back().points[i].data.size()  cob_3d_mapping_msgs::PlaneExtractionFeedback feedback_;
      cob_3d_mapping_msgs::PlaneExtractionResult result_;
      << " \t Step: " << sa.shapes.back().points[i].point_step << std::endl;
      }*/
  }
  hull_cloud->header = cloud->header;
  hull_cloud->height = 1;
  hull_cloud->width = hull_cloud->size();
  hull_cloud_dense->header = cloud->header;
  hull_cloud_dense->height = 1;
  hull_cloud_dense->width = hull_cloud_dense->size();
  pub_chull_.publish(hull_cloud);
  pub_chull_dense_.publish(hull_cloud_dense);
  pub_shape_array_.publish(sa);
  std::cout<<"[SN]-->sa array frame set to"<<sa.header.frame_id.c_str()<<"\n";
}


PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SegmentationAllInOneNodelet, cob_3d_segmentation::SegmentationAllInOneNodelet, nodelet::Nodelet);
