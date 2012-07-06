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
#include <boost/make_shared.hpp>

#include <pcl/io/io.h>
#include <pcl/ros/for_each_type.h>

#include <cob_3d_mapping_common/cylinder.h>



// Package includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
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

  chull_.setAlpha (0.2);

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
  publishShapeArray(graph_->clusters(), cloud);
  NODELET_INFO("Done with publishing .... ");
}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::publishShapeArray(ST::CH::Ptr cluster_handler, PointCloud::ConstPtr cloud)
{
  cob_3d_mapping_msgs::ShapeArray sa;
  sa.header = cloud->header;
  std::cout << "Frame: " << sa.header.frame_id << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (ST::CH::ClusterPtr c = cluster_handler->begin(); c != cluster_handler->end(); ++c)
  {
    switch(c->type)
    {
    case I_PLANE:
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
      PolygonContours<PolygonPoint> poly;
      pe_.outline(cloud->width, cloud->height, c->border_points, poly);
      //std::cout << "Polys: " << poly.polys_.size() << std::endl;
      if (!poly.polys_.size()) continue; // continue, if no contours were found

      int max_idx=0, max_size=0;
      for (int i = 0; i < poly.polys_.size(); ++i)
      {
        if (poly.polys_[i].size() > max_size) { max_idx = i; max_size = poly.polys_[i].size(); }
      }

      sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
      cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
      s->type = cob_3d_mapping_msgs::Shape::POLYGON;
      s->points.resize(poly.polys_.size());

      Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
      for (int i = 0; i < poly.polys_.size(); ++i)
      {
        if (i == max_idx) s->holes.push_back(false);
        else s->holes.push_back(true);
        for (std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin(); it != poly.polys_[i].end(); ++it)
        {
          pcl::PointXYZRGB p = cloud->points[PolygonPoint::getInd(it->x, it->y)];
          if (i==max_idx)
          {
            centroid += p.getVector3fMap();
          }
          hull_cloud->points.push_back(p);
          hull->points.push_back(p);
        }
        hull->height = 1;
        hull->width = hull->size();
        pcl::toROSMsg(*hull, s->points[i]);
        //std::cout << hull->points.size() << " | " << s->points[i].data.size() << " ";
        hull->clear();
      }
      //std::cout << std::endl;
      //std::cout << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << std::endl;
      centroid /= poly.polys_[max_idx].size();
      s->centroid.x = centroid[0];
      s->centroid.y = centroid[1];
      s->centroid.z = centroid[2];

      s->params.resize(4);
      Eigen::Vector3f orientation = c->pca_point_comp3;
      s->params[0] = orientation(0); // n_x
      s->params[1] = orientation(1); // n_y
      s->params[2] = orientation(2); // n_z
      s->params[3] = fabs(centroid.dot(orientation)); // d
      break;
    }
    case I_CYL:
    {

      //      cob_3d_mapping::CylinderPtr  cyl  =cob_3d_mapping::CylinderPtr(new cob_3d_mapping::Cylinder());
      //
      //      Eigen::Vector3f centroid3f  = c->getCentroid();
      //      cyl->centroid << centroid3f[0] , centroid3f[1] , centroid3f[2] , 0;
      //
      //      cyl->axes_.resize(3);
      //      cyl->axes_[1] = c->pca_inter_comp1;
      //
      //
      //      cyl->ParamsFromCloud(cloud,c->indices_);

      break;
    }
    default:
    {
      break;
    }
    }

    /*for (int i = 0; i < sa.shapes.back().points.size(); ++i)
    {
      std::cout << "Size: " << sa.shapes.back().points[i].data.size()
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
