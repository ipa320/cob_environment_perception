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
 * Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2011

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
#ifndef JUMP_EDGE_FILTER_HPP_
#define JUMP_EDGE_FILTER_HPP_

//##################
//#### includes ####

// opencv includes
#include <opencv/cv.h>

// pcl includes
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

// cob_env_model includes
#include "cob_env_model/cob_filters/jump_edge_filter.h"
#include <cob_env_model/cpc_point.h>

template <typename PointT> void
 cob_env_model::JumpEdgeFilter<PointT>::applyFilter (PointCloud &pc_out)
{
  //pointer to indices of points to be removed
  pcl::PointIndices::Ptr points_to_remove (new pcl::PointIndices ());

  //upper angle threshold in radians
  double upper_angle_thresh = upper_angle_deg_/180*M_PI;

  //Lower angle threshold in radians
  double lower_angle_thresh = (180-upper_angle_deg_)/180*M_PI;
  for (unsigned int i = 0; i < input_->points.size(); i++)
  {
    if(i< input_->width || i%input_->width==0 || i%input_->width==3 || i>input_->width*(input_->height-1)) continue; //skip border points
    Eigen::Vector3f v_m(input_->points[i].x,input_->points[i].y,input_->points[i].z);
    Eigen::Vector3f v_m_n = v_m.normalized();
    int index = i-input_->width-1;
    Eigen::Vector3f vd_ul(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_ul.normalize();

    double angle = std::acos(v_m_n.dot(vd_ul));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i-input_->width;
    Eigen::Vector3f vd_u(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_u.normalize();
    angle = std::acos(v_m_n.dot(vd_u));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i-input_->width+1;
    Eigen::Vector3f vd_ur(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_ur.normalize();
    angle = std::acos(v_m_n.dot(vd_ur));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i-1;
    Eigen::Vector3f vd_l(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_l.normalize();
    angle = std::acos(v_m_n.dot(vd_l));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i+1;
    Eigen::Vector3f vd_r(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_r.normalize();
    angle = std::acos(v_m_n.dot(vd_r));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i+input_->width-1;
    Eigen::Vector3f vd_ll(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_ll.normalize();
    angle = std::acos(v_m_n.dot(vd_ll));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i+input_->width;
    Eigen::Vector3f vd_lo(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_lo.normalize();
    angle = std::acos(v_m_n.dot(vd_lo));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
    index = i+input_->width+1;
    Eigen::Vector3f vd_lr(v_m(0)-input_->points[index].x, v_m(1)-input_->points[index].y, v_m(2)-input_->points[index].z);
    vd_lr.normalize();
    angle = std::acos(v_m_n.dot(vd_lr));
    if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back(i);
      continue;
    }
  }
  pcl::ExtractIndices<CPCPoint> extractIndices;
  extractIndices.setInputCloud(input_);
  extractIndices.setIndices(points_to_remove);
  extractIndices.setNegative(true);
  extractIndices.filter(pc_out);
 }

#define PCL_INSTANTIATE_JumpEdgeFilter(T) template class cob_env_model::JumpEdgeFilter<T>;
#endif /* JUMP_EDGE_FILTER_HPP_ */
