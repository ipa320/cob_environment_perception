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
#include <pcl/for_each_type.h>

#include <cob_3d_mapping_common/cylinder.h>



// Package includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_segmentation/segmentation_all_in_one_nodelet_TRAFO.h"


//Constructor





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
  pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("/plane_extraction/shape_array",1);
  pub_chull_ = nh_.advertise<PointCloud>("concave_hull", 1);
  std::cout << "Loaded segmentation nodelet" << std::endl;


}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::received_cloud_cb(PointCloud::ConstPtr cloud)
{
  tf::StampedTransform trf_map;
  try
  {
    tf_listener_.waitForTransform(target_frame_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(2));
    tf_listener_.lookupTransform(target_frame_, cloud->header.frame_id, cloud->header.stamp, trf_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("[segmentation] : %s",ex.what());
    return;
  }
  Eigen::Affine3d ad;
  tf::TransformTFToEigen(trf_map, ad);
  Eigen::Affine3f af = ad.cast<float>();

  //Eigen::Affine3f af = Eigen::Affine3f::Identity();

  //      cloud->header.frame_id =target_frame_;

  PrecisionStopWatch t;
  t.precisionStart();
  NODELET_INFO("Start with segmentation .... ");
  one_.setInputCloud(cloud);
  one_.compute(*normals_);

  *classified_ = *segmented_ = *cloud;

  seg_.setInputCloud(cloud);
  seg_.performInitialSegmentation();
  seg_.refineSegmentation();
  NODELET_INFO("Done with segmentation .... ");
  graph_->clusters()->mapClusterColor(segmented_);

  cc_.setPointCloudIn(cloud);
  cc_.classify();
  graph_->clusters()->mapTypeColor(classified_);
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
  publishShapeArray(graph_->clusters(), cloud, af);

  NODELET_INFO("Done with publishing .... ");

}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::publishShapeArray(
    ST::CH::Ptr cluster_handler, PointCloud::ConstPtr cloud, Eigen::Affine3f& tf)
{
  cob_3d_mapping_msgs::ShapeArray sa;
  sa.header = cloud->header;
  sa.header.frame_id = target_frame_.c_str();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (ST::CH::ClusterPtr c = cluster_handler->begin(); c != cluster_handler->end(); ++c)
  {
    // compute hull:
    if (c->getCentroid()[2] > 5.0f) continue;
    if (c->type != I_PLANE /*&& c->type != I_CYL*/) continue;
    if (c->size() <= ceil(1.1f * static_cast<float>(c->border_points.size())))
    {
      std::cout <<"[ " << c->size() <<" | "<< c->border_points.size() << " ]" << std::endl;
      continue;
    }
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
    s->points.resize(poly.polys_.size());
    s->header.frame_id = target_frame_.c_str();

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      if (i == max_idx) s->holes.push_back(false);
      else s->holes.push_back(true);
      pcl::PointXYZRGB p;
      for (std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin(); it != poly.polys_[i].end(); ++it)
      {
        p.getVector3fMap() = tf * cloud->points[PolygonPoint::getInd(it->x, it->y)].getVector3fMap();
        if (i==max_idx) { centroid += p.getVector3fMap(); }
        hull_cloud->points.push_back(p);
        hull->points.push_back(p);
      }
      hull->height = 1;
      hull->width = hull->points.size();
      pcl::toROSMsg(*hull, s->points[i]);
      hull->clear();
    }
    centroid /= poly.polys_[max_idx].size();
    Eigen::Vector3f tf_centroid = tf * c->getCentroid();//centroid;
    s->centroid.x = tf_centroid[0];
    s->centroid.y = tf_centroid[1];
    s->centroid.z = tf_centroid[2];

    // Set type specific parameters:
    switch(c->type)
    {
    case I_PLANE:
    {
      //std::cout << "Plane: " << c->size() << ", " << c->border_points.size() << std::endl;
      s->type = cob_3d_mapping_msgs::Shape::POLYGON;

      s->params.resize(4);
      Eigen::Vector3f orientation = tf.rotation() * c->pca_point_comp3;
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

      cyl->axes_.resize(3);
      cyl->axes_[1] =  c->pca_inter_comp1;
      cyl->ParamsFromCloud(cloud,c->indices_);


      //write parameters to msg - after transformation to target frame
      Eigen::Vector3f tf_axes_1 = tf.rotation() * cyl->axes_[1];
      s->params[0] =  tf_axes_1[0];
      s->params[1] = tf_axes_1[1];
      s->params[2] = tf_axes_1[2];

      Eigen::Vector3f tf_axes_2 = tf.rotation() * cyl->axes_[2];

      s->params[3] = tf_axes_2[0];
      s->params[4] = tf_axes_2[1];
      s->params[5] = tf_axes_2[2];


      Eigen::Vector3f tf_origin = tf * cyl->origin_;
      s->params[6] =  tf_origin[0];
      s->params[7] =  tf_origin[1];
      s->params[8] =  tf_origin[2];

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
  pub_chull_.publish(hull_cloud);
  pub_shape_array_.publish(sa);
}


PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SegmentationAllInOneNodelet, cob_3d_segmentation::SegmentationAllInOneNodelet, nodelet::Nodelet);
