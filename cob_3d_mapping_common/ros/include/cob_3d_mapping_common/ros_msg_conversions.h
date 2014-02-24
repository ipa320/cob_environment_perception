/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 03/2012
*
* \brief
* Conversion functions between shape message and class representations
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef COB_3D_MAPPING_ROS_MSG_CONV_H_
#define COB_3D_MAPPING_ROS_MSG_CONV_H_

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cob_3d_mapping_msgs/Shape.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_common/cylinder.h"
#include <eigen_conversions/eigen_msg.h>

namespace cob_3d_mapping
{
  //TODO: purpose? where used? is this the correct place?
  // should work for cob_3d_mapping_msgs::Shape, Polygon, Cylinder
  template<typename ShapeT>
  inline int getHullIndex(const ShapeT& s)
  {
    int idx = -1;
    for(size_t i=0; i<s.holes.size(); ++i) { if(!s.holes[i]) { idx=i; break; } }
    return idx;
  }

  template<typename PointT>
  inline bool shape2hull(const cob_3d_mapping_msgs::Shape& s, pcl::PointCloud<PointT>& hull)
  {
    int idx = getHullIndex(s);
    if (idx<0)
    {
      std::cerr <<  "[shape2hull()]: ERROR! No positive contour!" << std::endl;
      return false;
    }
    pcl::PCLPointCloud2 hull2;
    pcl_conversions::toPCL(s.points[idx], hull2);
    pcl::fromPCLPointCloud2(hull2, hull);
    //pcl::fromROSMsg(s.points[idx], hull);
    return true;
  }

  /**
   * @brief writing to a ros message to convert a feature map
   *
   * writing to a ros message to convert a feature map
   *
   * @param p ros message containing polygons
   * @param p input as feature map
   *
   * @return nothing
   */
  inline void
  toROSMsg(const Polygon& p, cob_3d_mapping_msgs::Shape& s)
  {
    s.id = p.id_;
    s.type = cob_3d_mapping_msgs::Shape::POLYGON;
    s.params.resize(4);
    s.params[0] = p.normal_(0);
    s.params[1] = p.normal_(1);
    s.params[2] = p.normal_(2);
    s.params[3] = p.d_;
    tf::poseEigenToMsg(p.pose_.cast<double>(), s.pose);
    //s.centroid.x = p.centroid(0);
    //s.centroid.y = p.centroid(1);
    //s.centroid.z = p.centroid(2);
    s.color.r = p.color_[0];
    s.color.g = p.color_[1];
    s.color.b = p.color_[2];
    s.color.a = p.color_[3];
    s.weight = p.merge_weight_;

    s.points.resize(p.contours_.size());
    s.holes.resize(p.holes_.size());

    for(unsigned int i=0; i < p.contours_.size(); i++)
    {
      s.holes[i] = p.holes_[i];
      //s.points[i].points.resize(p.contours[i].size());
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for(unsigned int j=0; j<p.contours_[i].size(); j++)
      {
        pcl::PointXYZ pt;
        pt.x = p.contours_[i][j](0);
        pt.y = p.contours_[i][j](1);
        pt.z = 0;//p.contours[i][j](2);
        cloud.points.push_back(pt);
        /*s.points[i].points[j].x = p.contours[i][j](0);
        s.points[i].points[j].y = p.contours[i][j](1);
        s.points[i].points[j].z = p.contours[i][j](2);*/
      }
      pcl::PCLPointCloud2 cloud2;
      pcl::toPCLPointCloud2(cloud, cloud2);
      sensor_msgs::PointCloud2 cloud_msg;
      pcl_conversions::fromPCL(cloud2, cloud_msg);
      //pcl::toROSMsg(cloud, cloud_msg);
      s.points[i]= cloud_msg;
    }
  }

  /**
   * @brief writing to a ros message to convert a feature map
   *
   * writing to a ros message to convert a feature map
   *
   * @param p ros message containing polygons
   * @param p input as feature map
   *
   * @return nothing
   */
  inline bool
  fromROSMsg(const cob_3d_mapping_msgs::Shape& s, Polygon& p)
  {
    p.id_ = s.id;
    /*p.centroid(0) = s.centroid.x;
    p.centroid(1) = s.centroid.y;
    p.centroid(2) = s.centroid.z;*/
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(s.pose, pose);
    p.pose_ = pose.cast<float>();
    p.normal_(0) = s.params[0];
    p.normal_(1) = s.params[1];
    p.normal_(2) = s.params[2];
    p.d_ = s.params[3];
    //p.merged_ = 1;
    p.color_[0] = s.color.r;
    p.color_[1] = s.color.g;
    p.color_[2] = s.color.b;
    p.color_[3] = s.color.a;
    p.merge_weight_ = s.weight;
    //p.contours.resize(p.polygons.size());
    for(unsigned int i=0; i<s.points.size(); i++)
    {
      p.holes_.push_back(s.holes[i]);
      if(s.points[i].data.size())
      {
        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(s.points[i], cloud2);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);
        //pcl::fromROSMsg(s.points[i], cloud);
        std::vector<Eigen::Vector2f> pts;
        pts.resize(cloud.points.size());
        for(unsigned int j=0; j<cloud.points.size(); j++)
        {
          pts[j](0) = cloud.points[j].x;
          pts[j](1) = cloud.points[j].y;
          //pts[j](2) = cloud.points[j].z;
        }
        p.contours_.push_back(pts);
        //pcl::getTransformationFromTwoUnitVectorsAndOrigin(
        //    p.normal_.unitOrthogonal(),p.normal_,p.centroid.head(3),p.transform_from_world_to_plane);
      }
      else
      {
        std::cout << "Polygon has no points" << std::endl;
        return false;
      }
    }
    return true;
  }

  inline void
  toROSMsg(const Cylinder& c, cob_3d_mapping_msgs::Shape& s)
  {
    s.params.resize(6);
    /*s.params[0]=c.normal_[0];
    s.params[1]=c.normal_[1];
    s.params[2]=c.normal_[2];*/
    s.params[0] = c.sym_axis_[0];
    s.params[1] = c.sym_axis_[1];
    s.params[2] = c.sym_axis_[2];
    /*s.params[6]=c.origin_[0];
    s.params[7]=c.origin_[1];
    s.params[8]=c.origin_[2];*/
    s.params[3] = c.r_;
    s.params[4] = c.h_min_;
    s.params[5] = c.h_max_;
    tf::poseEigenToMsg(c.pose_.cast<double>(), s.pose);
    s.id = c.id_;
    s.type = cob_3d_mapping_msgs::Shape::CYLINDER;
    s.weight = c.merge_weight_;
    s.color.r = c.color_[0];
    s.color.g = c.color_[1];
    s.color.b = c.color_[2];
    s.color.a = c.color_[3];
    s.points.resize(c.contours_.size());
    s.holes.resize(c.holes_.size());

    for(unsigned int i=0; i<c.contours_.size(); i++)
    {
      s.holes[i] = c.holes_[i];
      //s.points[i].points.resize(p.contours[i].size());
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for(unsigned int j=0; j<c.contours_[i].size(); j++)
      {
        pcl::PointXYZ pt;
        pt.x = c.contours_[i][j](0);
        pt.y = c.contours_[i][j](1);
        pt.z = 0;
        cloud.points.push_back(pt);
        //std::cout << pt.x << "," << pt.y << std::endl;
      }
      pcl::PCLPointCloud2 cloud2;
      pcl::toPCLPointCloud2(cloud, cloud2);
      sensor_msgs::PointCloud2 cloud_msg;
      pcl_conversions::fromPCL(cloud2, cloud_msg);
      //pcl::toROSMsg(cloud, cloud_msg);
      s.points[i]= cloud_msg;
    }
  }

  inline bool
  fromROSMsg(const cob_3d_mapping_msgs::Shape& s,Cylinder& c)
  {
    c.id_ = s.id;
    /*c.normal_[0] = s.params[0];
    c.normal_[1] = s.params[1];
    c.normal_[2] = s.params[2];*/
    c.sym_axis_[0]= s.params[0];
    c.sym_axis_[1] = s.params[1];
    c.sym_axis_[2] = s.params[2];
    /*c.origin_[0] = s.params[6];
    c.origin_[1] = s.params[7];
    c.origin_[2] = s.params[8];*/
    c.r_ = s.params[3];
    c.h_min_ = s.params[4];
    c.h_max_ = s.params[5];
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(s.pose, pose);
    c.pose_ = pose.cast<float>();
    //c.merged_ = 1;
    c.merge_weight_ = s.weight;
    c.color_[0] = s.color.r;
    c.color_[1] = s.color.g;
    c.color_[2] = s.color.b;
    c.color_[3] = s.color.a;

    for(unsigned int i=0; i<s.points.size(); i++)
    {
      c.holes_.push_back(s.holes[i]);
      if(s.points[i].data.size())
      {
        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(s.points[i], cloud2);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);
        //pcl::fromROSMsg(s.points[i], cloud);
        std::vector<Eigen::Vector2f> pts;
        pts.resize(cloud.points.size());
        for(unsigned int j=0; j<cloud.points.size(); j++)
        {
          pts[j](0) = cloud.points[j].x;
          pts[j](1) = cloud.points[j].y;
          //pts[j](2) = 0;
        }
        c.contours_.push_back(pts);
      }
      else
      {
        std::cout << "Cylinder has no points" << std::endl;
        return false;
      }
    }
    return true;
  }
}

#endif

