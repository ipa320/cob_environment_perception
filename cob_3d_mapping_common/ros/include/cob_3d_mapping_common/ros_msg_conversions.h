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
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
 * ToDo:
 *
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

#ifndef ROS_MSG_CONV_H_
#define ROS_MSG_CONV_H_

#include <cob_3d_mapping_msgs/Shape.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_common/cylinder.h"

namespace cob_3d_mapping
{
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
    s.id = p.id;
    s.type = cob_3d_mapping_msgs::Shape::POLYGON;
    s.params.resize(4);
    s.params[0] = p.normal(0);
    s.params[1] = p.normal(1);
    s.params[2] = p.normal(2);
    s.params[3] = p.d;
    s.centroid.x = p.centroid(0);
    s.centroid.y = p.centroid(1);
    s.centroid.z = p.centroid(2);
    s.color.r = p.color[0];
    s.color.g = p.color[1];
    s.color.b = p.color[2];
    s.color.a = p.color[3];
    s.points.resize(p.contours.size());
    s.holes.resize(p.holes.size());
    for(unsigned int i=0; i<p.contours.size(); i++)
    {
      s.holes[i] = p.holes[i];
      //s.points[i].points.resize(p.contours[i].size());
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for(unsigned int j=0; j<p.contours[i].size(); j++)
      {
        pcl::PointXYZ pt;
        pt.x = p.contours[i][j](0);
        pt.y = p.contours[i][j](1);
        pt.z = p.contours[i][j](2);
        cloud.points.push_back(pt);
        /*s.points[i].points[j].x = p.contours[i][j](0);
        s.points[i].points[j].y = p.contours[i][j](1);
        s.points[i].points[j].z = p.contours[i][j](2);*/
      }
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
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
    p.id = s.id;
    p.centroid(0) = s.centroid.x;
    p.centroid(1) = s.centroid.y;
    p.centroid(2) = s.centroid.z;
    p.normal(0) = s.params[0];
    p.normal(1) = s.params[1];
    p.normal(2) = s.params[2];
    p.d = s.params[3];
    //std::cout << "normal: " << p.normal(0) << ","  << p.normal(1) << "," << p.normal(2) << std::endl;
    //std::cout << "d: " << p.d << std::endl << std::endl;
    p.merged = 1;
    p.color[0] = s.color.r;
    p.color[1] = s.color.g;
    p.color[2] = s.color.b;
    p.color[3] = s.color.a;
    //p.contours.resize(p.polygons.size());
    for(unsigned int i=0; i<s.points.size(); i++)
    {
      p.holes.push_back(s.holes[i]);
      if(s.points[i].data.size())
      {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(s.points[i], cloud);
        std::vector<Eigen::Vector3f> pts;
        pts.resize(cloud.points.size());
        for(unsigned int j=0; j<cloud.points.size(); j++)
        {
          /*pts[j] = Eigen::Vector3f(p.polygons[i].points[j].x,
                                   p.polygons[i].points[j].y,
                                   p.polygons[i].points[j].z);*/
          pts[j](0) = cloud.points[j].x;
          pts[j](1) = cloud.points[j].y;
          pts[j](2) = cloud.points[j].z;
        }
        p.contours.push_back(pts);
      }
      else
      {
        std::cout << "shape has no points" << std::endl;
        return false;
      }
    }
    return true;
  }

  inline void
  toROSMsg(const Cylinder& c, cob_3d_mapping_msgs::Shape& s)
  {


    s.params.resize(10);
    // x axis
    s.params[0]=c.sym_axis[0];
    s.params[1]=c.sym_axis[1];
    s.params[2]=c.sym_axis[2];


    s.params[3]=c.normal[0];
    s.params[4]=c.normal[1];
    s.params[5]=c.normal[2];


    s.params[6]=c.origin_[0];
    s.params[7]=c.origin_[1];
    s.params[8]=c.origin_[2];

    s.params[9]=c.r_;

    s.centroid.x = c.centroid[0];
    s.centroid.y = c.centroid[1];
    s.centroid.z = c.centroid[2];





    //std::cout << "normal: " << p.normal(0) << "," << p.normal(1) << "," << p.normal(2) << std::endl;
    //std::cout << "d: " << p.d << std::endl << std::endl;

    s.id=c.id;

    s.color.r=c.color[0];
    s.color.g=c.color[1];
    s.color.b=c.color[2];
    s.color.a=c.color[3];


    s.points.resize(c.contours.size());
    s.holes.resize(c.holes.size());
    for(unsigned int i=0; i<c.contours.size(); i++)
    {
      s.holes[i] = c.holes[i];
      //s.points[i].points.resize(p.contours[i].size());
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for(unsigned int j=0; j<c.contours[i].size(); j++)
      {
        pcl::PointXYZ pt;
        pt.x = c.contours[i][j](0);
        pt.y = c.contours[i][j](1);
        pt.z = c.contours[i][j](2);
        cloud.points.push_back(pt);

      }
      sensor_msgs::PointCloud2 cloud_msg;

      pcl::toROSMsg(cloud, cloud_msg);
      s.points[i]= cloud_msg;
    }




  }



  inline bool
  fromROSMsg(const cob_3d_mapping_msgs::Shape& s,Cylinder& c){



    c.id = 0;
    // c.centroid(0) = s.centroid.x;
    // c.centroid(1) = s.centroid.y;
    // c.centroid(2) = s.centroid.z;

    c.sym_axis[0]= s.params[0];
    c.sym_axis[1] = s.params[1];
    c.sym_axis[2] = s.params[2];

    c.normal[0]= s.params[3];
    c.normal[1] = s.params[4];
    c.normal[2] = s.params[5];


    c.origin_[0] = s.params[6];
    c.origin_[1] = s.params[7];
    c.origin_[2] = s.params[8];
    c.r_ = s.params[9];

    c.centroid[0]=s.centroid.x;
    c.centroid[1]=s.centroid.y;
    c.centroid[2]=s.centroid.z;




    //std::cout << "normal: " << p.normal(0) << "," << p.normal(1) << "," << p.normal(2) << std::endl;
    //std::cout << "d: " << p.d << std::endl << std::endl;
    c.merged = 1;
    c.color[0] = s.color.r;
    c.color[1] = s.color.g;
    c.color[2] = s.color.b;
    c.color[3] = s.color.a;
    //p.contours.resize(p.polygons.size());
    for(unsigned int i=0; i<s.points.size(); i++)
    {
      c.holes.push_back(s.holes[i]);
      if(s.points[i].data.size())
      {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(s.points[i], cloud);


        std::vector<Eigen::Vector3f> pts;
        pts.resize(cloud.points.size());
        for(unsigned int j=0; j<cloud.points.size(); j++)
        {
          /*pts[j] = Eigen::Vector3f(p.polygons[i].points[j].x,
p.polygons[i].points[j].y,
p.polygons[i].points[j].z);*/
          pts[j](0) = cloud.points[j].x;
          pts[j](1) = cloud.points[j].y;
          pts[j](2) = cloud.points[j].z;
        }
        c.contours.push_back(pts);
      }
      else
      {
        std::cout << "cylinder has no points" << std::endl;
        return false;
      }
    }
    return true;



  }




}

#endif

