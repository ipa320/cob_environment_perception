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
 * \date Date of creation: 05/2012
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

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <boost/make_shared.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
  #ifdef PCL_VERSION_COMPARE //fuerte
    #include <pcl/for_each_type.h>
  #else //electric
    #include <pcl/ros/for_each_type.h>
  #endif

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// Package includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_common/cylinder.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include "cob_3d_segmentation/segmentation_all_in_one_nodelet.h"


void
cob_3d_segmentation::SegmentationAllInOneNodelet::onInit()
{
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

  if(!is_running_ && !enable_action_mode_)
  {
    ROS_INFO("Starting segmentation...");
    sub_points_ = nh_.subscribe("cloud_in", 1, &SegmentationAllInOneNodelet::receivedCloudCallback, this);
    is_running_ = true;
  }
  pub_segmented_ = nh_.advertise<PointCloud>("/segmentation/segmented_cloud", 1);
  pub_classified_ = nh_.advertise<PointCloud>("/segmentation/classified_cloud", 1);
  pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("/segmentation/shape_array",1);
  pub_chull_ = nh_.advertise<PointCloud>("/segmentation/concave_hull", 1);
  pub_chull_dense_ = nh_.advertise<PointCloud>("/segmentation/concave_hull_dense", 1);
  std::cout << "Loaded segmentation nodelet" << std::endl;

  as_ = new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerAction>(nh_, "segmentation/trigger", boost::bind(&SegmentationAllInOneNodelet::actionCallback, this, _1), false);
  as_->start();
}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::configCallback(
    cob_3d_segmentation::segmentation_nodeletConfig& config,
    uint32_t level)
{
  NODELET_INFO("[segmentation]: received new parameters");
  centroid_passthrough_ = config.centroid_passthrough;
  enable_action_mode_ = config.enable_action_mode;
}


void
cob_3d_segmentation::SegmentationAllInOneNodelet::actionCallback(
  const cob_3d_mapping_msgs::TriggerGoalConstPtr& goal)
{
  cob_3d_mapping_msgs::TriggerResult result;
  if(goal->start && !is_running_)
  {
    ROS_INFO("Starting segmentation...");
    sub_points_ = nh_.subscribe("cloud_in", 1, &SegmentationAllInOneNodelet::receivedCloudCallback, this);
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
cob_3d_segmentation::SegmentationAllInOneNodelet::receivedCloudCallback(PointCloud::ConstPtr cloud)
{
  PrecisionStopWatch t;
  t.precisionStart();
  NODELET_INFO("Start with segmentation ... ");
  one_.setInputCloud(cloud);
  one_.compute(*normals_);

  *classified_ = *segmented_ = *cloud;

  seg_.setInputCloud(cloud);
  seg_.performInitialSegmentation();
  seg_.refineSegmentation();
  graph_->clusters()->mapClusterColor(segmented_);

  //seg_.getPotentialObjects(objects, 500);
  //std::cout << "Found " << objects.size() << " potentail objects" << std::endl;
  //NODELET_INFO("Done with segmentation .... ");

  cc_.setPointCloudIn(cloud);
  cc_.classify();
  graph_->clusters()->mapTypeColor(classified_);
  graph_->clusters()->mapClusterBorders(classified_);
  
  NODELET_INFO("Segmentation took %f s.", t.precisionStop());
  
  //NODELET_INFO("publish first cloud .... ");
  pub_segmented_.publish(segmented_);
  //NODELET_INFO("publish second cloud .... ");
  pub_classified_.publish(classified_);
  //NODELET_INFO("publish shape array .... ");
  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bp (new pcl::PointCloud<pcl::PointXYZRGB>);
   *bp = *cloud;
  graph_->clusters()->mapClusterBorders(bp);
  std::stringstream ss;
  ss << "/share/goa-sf/pcd_data/bags/pcd_borders/borders_"<<cloud->header.stamp<<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *bp);
   */
  publishShapeArray(graph_->clusters(), cloud);

  NODELET_INFO("Done with publishing .... ");

}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::publishShapeArray(
  ST::CH::Ptr cluster_handler, PointCloud::ConstPtr cloud)
{
  cob_3d_mapping_msgs::ShapeArray sa;
  pcl_conversions::fromPCL(cloud->header, sa.header);
  //sa.header = cloud->header;
  //sa.header.frame_id = cloud->header.frame_id.c_str();
  //std::cout<<"[SN]-->CLOUD FRAME"<<cloud->header.frame_id.c_str()<<"\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud_dense(new pcl::PointCloud<pcl::PointXYZRGB>);

  unsigned int id = 0;
  for (ST::CH::ClusterPtr c = cluster_handler->begin(); c != cluster_handler->end(); ++c)
  {
    // compute hull:
    if (c->getCentroid()[2] > centroid_passthrough_) continue;
    if (c->type != I_PLANE && c->type != I_CYL) continue;
    if (c->size() <= ceil(1.1f * static_cast<float>(c->border_points.size())))
    {
      //std::cout <<"[ " << c->size() <<" | "<< c->border_points.size() << " ]" << std::endl;
      continue;
    }

    for (size_t i = 0; i < c->border_points.size(); ++i)
      hull_cloud_dense->push_back((*segmented_)[c->border_points[i].y * segmented_->width + c->border_points[i].x]);

    PolygonContours<PolygonPoint> poly;
    //std::cout << "Get outline for " << c->size() << " Points with "<< c->border_points.size() << " border points" << std::endl;
    pe_.outline(cloud->width, cloud->height, c->border_points, poly);
    if (!poly.polys_.size()) continue; // continue, if no contours were found
    //std::cout << "border -> outline: " << c->border_points.size() << " -> " << poly.polys_[0].size() << std::endl;

    int max_idx=0, max_size=0;
    //std::cout << "Polys: ";
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      //std::cout << poly.polys_[i].size() << " ";
      if ((int)poly.polys_[i].size() > max_size) { max_idx = i; max_size = poly.polys_[i].size(); }
    }
    //std::cout << std::endl;

    sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
    cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
    //s->id = id++;
    //s->points.resize(poly.polys_.size());
    s->header.frame_id = cloud->header.frame_id.c_str();
    Eigen::Vector3f color_tmp = c->computeDominantColorVector().cast<float>();
    float temp_inv = 1.0f/255.0f;
    std::vector<float> color(4,1);
    color[0] = color_tmp(0) * temp_inv;
    color[1] = color_tmp(1) * temp_inv;
    color[2] = color_tmp(2) * temp_inv;
    /*s->color.r = color(0) * temp_inv;
    s->color.g = color(1) * temp_inv;
    s->color.b = color(2) * temp_inv;
    s->color.a = 1.0f;*/
    //Eigen::Vector3f origin = c->getCentroid();
    //    Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - c->pca_point_comp3 * c->pca_point_comp3.transpose(); // projection
    /*for (int i = 0; i < (int)poly.polys_.size(); ++i)
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
    }*/
    /*Eigen::Affine3f pose_inv;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(
        normal.unitOrthogonal(), normal, centroid, pose_inv);
    tf::poseEigenToMsg(pose.inverse().cast<double>(), s->pose_inv.inverse());*/
    /*s->centroid.x = centroid[0];
    s->centroid.y = centroid[1];
    s->centroid.z = centroid[2];*/

    std::vector<pcl::PointCloud<pcl::PointXYZ> > contours_3d;
    std::vector<bool> holes;
    //pcl::transformPointCloud(*cloud, cloud_tr, pose.inverse());
    for (int i = 0; i < (int)poly.polys_.size(); ++i)
    {
      pcl::PointCloud<pcl::PointXYZ> contour;
      if (i == max_idx)
      {
        holes.push_back(false);
        std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin();
        for ( ; it != poly.polys_[i].end(); ++it) {
          pcl::PointXYZ pt;
          pt.getVector3fMap() = cloud->points[ it->x + it->y * cloud->width ].getVector3fMap();
          contour.push_back( pt );
          //contour.push_back( cloud->points[ it->x + it->y * cloud->width ] );
        }
      }
      else
      {
        holes.push_back(true);
        std::vector<PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin();
        for ( ; it != poly.polys_[i].rend(); ++it) {
          pcl::PointXYZ pt;
          pt.getVector3fMap() = cloud->points[ it->x + it->y * cloud->width ].getVector3fMap();
          contour.push_back( pt );
          //contour.push_back( cloud->points[ it->x + it->y * cloud->width ] );
        }
      }
      contour.height = 1;
      contour.width = contour.size();
      contours_3d.push_back(contour);
      //pcl::toROSMsg(*hull, s->points[i]);
      //hull->clear();
    }

    // Set type specific parameters:
    switch(c->type)
    {
    case I_PLANE:
    {
      cob_3d_mapping::Polygon::Ptr  poly(new cob_3d_mapping::Polygon(id,
                                                                     c->pca_point_comp3,
                                                                     c->getCentroid()/*fabs(c->getCentroid().dot(c->pca_point_comp3))*/,
                                                                     contours_3d,
                                                                     holes,
                                                                     color));
      cob_3d_mapping::toROSMsg(*poly, *s);
      //std::cout << "Plane: " << c->size() << ", " << c->border_points.size() << std::endl;
      /*s->type = cob_3d_mapping_msgs::Shape::POLYGON;

      s->params.resize(4);
      Eigen::Vector3f orientation =  c->pca_point_comp3;
      s->params[0] = orientation(0); // n_x
      s->params[1] = orientation(1); // n_y
      s->params[2] = orientation(2); // n_z
      s->params[3] = fabs(centroid.dot(orientation)); // d*/
      break;
    }
    case I_CYL:
    {
      /*Cylinder(unsigned int id,
                   Eigen::Vector3f origin,
                   Eigen::Vector3f sym_axis,
                   double radius,
                   std::vector<pcl::PointCloud<pcl::PointXYZ> >& contours_3d,
                   std::vector<bool> holes,
                   std::vector<float> color*/
      //std::cout<<"CLYINDER is published\n";
      //s->type = cob_3d_mapping_msgs::Shape::CYLINDER;
      //s->params.resize(6);
      Eigen::Vector3f origin = c->getCentroid();
      double radius = cob_3d_mapping::radiusAndOriginFromCloud(cloud, c->indices_, origin, c->pca_inter_comp1);
      cob_3d_mapping::Cylinder::Ptr cyl = cob_3d_mapping::Cylinder::Ptr(new cob_3d_mapping::Cylinder(id,
                                                                                                      origin,
                                                                                                      c->pca_inter_comp1,
                                                                                                      radius,
                                                                                                      contours_3d,
                                                                                                      holes,
                                                                                                      color));
      //Eigen::Vector3f centroid3f  = c->getCentroid();
      //cyl->centroid << centroid3f[0] , centroid3f[1] , centroid3f[2] , 0;

      //cyl->sym_axis_ =  c->pca_inter_comp1;
      //cyl->updateAttributes(c->pca_inter_comp1, centroid3f);
      //std::cout<<"sym axis\n"<<cyl->sym_axis<<"\n";
      //std::cout<<"centroid\n"<<cyl->centroid<<"\n";
      //cyl->ParamsFromCloud(cloud,c->indices_);
      //cyl->r_ = radiusFromCloud(cloud,c->indices_);

      cob_3d_mapping::toROSMsg(*cyl, *s);
      //pcl::io::savePCDFile("/tmp/cyl_cont.pcd", contours_3d[0]);
      //write parameters to msg - after transformation to target frame
      /*s->params[0] = cyl->normal[0];
      s->params[1] = cyl->normal[1];
      s->params[2] = cyl->normal[2];

      s->params[3] = cyl->sym_axis[0];
      s->params[4] = cyl->sym_axis[1];
      s->params[5] = cyl->sym_axis[2];


      s->params[6] = cyl->origin_[0];
      s->params[7] = cyl->origin_[1];
      s->params[8] = cyl->origin_[2];

      s->params[9]= cyl->r_;*/

      break;
    }
    default:
    {
      break;
    }
    }
    id++;
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
  //std::cout<<"[SN]-->sa array frame set to"<<sa.header.frame_id.c_str()<<"\n";
}


PLUGINLIB_EXPORT_CLASS(cob_3d_segmentation::SegmentationAllInOneNodelet, nodelet::Nodelet);
