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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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
//##################
//#### includes ####

//opencv includes
#include <opencv/cv.h>

// PCL includes
#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

// cob_env_model includes
#include "cob_env_model/point_types.h"
#include "cob_env_model/filters/jump_edge_filter.h"
#include "cob_env_model/filters/impl/jump_edge_filter.hpp"

void
cob_env_model::JumpEdgeFilter<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &pc_out)
{
  //pc_out.header = input_->header;
  /* pc_out.fields = input_->fields;
  pc_out.point_step = input_->point_step;
  pc_out.data.resize (input_->data.size ());*/
  pcl::PointIndices::Ptr points_to_remove (new pcl::PointIndices ());
  double upper_angle_thresh = upper_angle_deg_ / 180 * M_PI;
  double lower_angle_thresh = (180 - 170.0) / 180 * M_PI;

  unsigned int x_offset = 0;
  unsigned int y_offset = 0;
  unsigned int z_offset = 0;
  for (size_t d = 0; d < input_->fields.size (); ++d)
  {
    if (input_->fields[d].name == "x")
      x_offset = input_->fields[d].offset;
    if (input_->fields[d].name == "y")
      y_offset = input_->fields[d].offset;
    if (input_->fields[d].name == "z")
      z_offset = input_->fields[d].offset;
  }

  const unsigned int total_points = input_->width * input_->height;
  //std::cout << " total points " << total_points << std::endl;

  for (unsigned int i = 0; i < total_points; i++)
  {
    if (i < input_->width || i % input_->width == 0 || i % input_->width == 3 || i > input_->width * (input_->height
        - 1))
      continue; //skip border points
    Eigen::Vector3f v_m (*(float*)&input_->data[i * input_->point_step + x_offset],
                         *(float*)&input_->data[i* input_->point_step + y_offset],
                         *(float*)&input_->data[i * input_->point_step + z_offset]);
    Eigen::Vector3f v_m_n = v_m.normalized ();
    int index = i - input_->width - 1;
    Eigen::Vector3f vd_ul (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                           v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                           v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);

    vd_ul.normalize ();
    double angle = std::acos (v_m_n.dot (vd_ul));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i - input_->width;
    Eigen::Vector3f vd_u (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                          v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                          v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_u.normalize ();
    angle = std::acos (v_m_n.dot (vd_u));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i - input_->width + 1;
    Eigen::Vector3f vd_ur (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                           v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                           v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_ur.normalize ();
    angle = std::acos (v_m_n.dot (vd_ur));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i - 1;
    Eigen::Vector3f vd_l (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                          v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                          v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_l.normalize ();
    angle = std::acos (v_m_n.dot (vd_l));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i + 1;
    Eigen::Vector3f vd_r (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                          v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                          v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_r.normalize ();
    angle = std::acos (v_m_n.dot (vd_r));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i + input_->width - 1;
    Eigen::Vector3f vd_ll (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                           v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                           v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_ll.normalize ();
    angle = std::acos (v_m_n.dot (vd_ll));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i + input_->width;
    Eigen::Vector3f vd_lo (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                           v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                           v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_lo.normalize ();
    angle = std::acos (v_m_n.dot (vd_lo));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
    index = i + input_->width + 1;
    Eigen::Vector3f vd_lr (v_m (0) - *(float*)&input_->data[index * input_->point_step + x_offset],
                           v_m (1) - *(float*)&input_->data[index * input_->point_step + y_offset],
                           v_m (2) - *(float*)&input_->data[index * input_->point_step + z_offset]);
    vd_lr.normalize ();
    angle = std::acos (v_m_n.dot (vd_lr));
    if (angle > upper_angle_thresh || angle < lower_angle_thresh)
    {
      points_to_remove->indices.push_back (i);
      continue;
    }
  }

  pcl::ExtractIndices<sensor_msgs::PointCloud2> extractIndices;
  extractIndices.setInputCloud (input_);
  extractIndices.setIndices (points_to_remove);
  extractIndices.setNegative (true);
  extractIndices.filter (pc_out);
  points_to_remove->indices.clear ();
}

using namespace pcl;
PCL_INSTANTIATE(ExtractIndices, (PointXYZCI));
PCL_INSTANTIATE(ExtractIndices, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(JumpEdgeFilter, (PointXYZCI));
PCL_INSTANTIATE(JumpEdgeFilter, PCL_XYZ_POINT_TYPES);
