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
#ifndef SPECKLE_FILTER_HPP_
#define SPECKLE_FILTER_HPP_

//##################
//#### includes ####


#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"

// cob_env_model includes
#include "cob_env_model/cob_filters/speckle_filter.h"
#include "cob_env_model/cob_filters/impl/speckle_filter.hpp"
#include <cob_env_model/cpc_point.h>

#include <opencv/cv.h>
#include <cob_vision_utils/VisionUtils.h>

void
cob_env_model::SpeckleFilter<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &pc_out)
{
  /*
  cv::Mat xyz_mat_32F3 = cv::Mat (input_->height, input_->width, CV_32FC3);

  float* f_ptr = 0;
  int pc_msg_idx = 0;
  for (int row = 0; row < xyz_mat_32F3.rows; row++)
  {
    f_ptr = xyz_mat_32F3.ptr<float> (row);
    for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
    {
      memcpy (&f_ptr[3 * col], &input_->points[pc_msg_idx].x, 3 * sizeof(float));
    }
  }

  //FilterSpeckles (xyz_mat_32F3);
  cv::Mat buf;
  ipa_Utils::FilterSpeckles(xyz_mat_32F3, speckle_size_,speckle_range_, buf);

  pc_msg_idx = 0;
  for (int row = 0; row < xyz_mat_32F3.rows; row++)
  {
    f_ptr = xyz_mat_32F3.ptr<float> (row);
    for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
    {
      memcpy (&pc_out->points[pc_msg_idx].x, &f_ptr[3 * col], 3 * sizeof(float));
    }
  }
  */
}

using namespace pcl;
PCL_INSTANTIATE(SpeckleFilter, (CPCPoint));
