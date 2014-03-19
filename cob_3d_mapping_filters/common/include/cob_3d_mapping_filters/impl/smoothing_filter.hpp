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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_filters
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
#ifndef INTENSITY_FILTER_HPP_
#define INTENSITY_FILTER_HPP_

//##################
//#### includes ####

// cob_3d_mapping_filters includes
#include "cob_3d_mapping_filters/smoothing_filter.h"

template<typename PointT>
  void
  cob_3d_mapping_filters::SmoothingFilter<PointT>::applyFilter (PointCloud &pc_out)
  {
    // set the parameters for output poincloud (pc_out)
    pc_out.points.resize (input_->points.size ());
    pc_out.header = input_->header;

    //resize pc_out according to filtered points
    pc_out.width = input_->width;
    pc_out.height = input_->height;
    pc_out.is_dense = input_->is_dense;

    Eigen::Matrix3f F;
    F.fill (smoothing_factor_);
    F (1, 1) = 1;
    const float sum = F.sum ();

    //Go through all points and discard points with intensity value above filter limit
    for (unsigned int x = 0; x < input_->width; x++)
      for (unsigned int y = 0; y < input_->height; y++)
      {
        float z = (*input_) (x, y).z;
        float nz = 0;
        for (int a = 0; a < 3; a++)
          for (int b = 0; b < 3; b++)
            if ((x + a - 1) < input_->width && (y + b - 1) < input_->height
                && std::abs ((*input_) (x + a - 1, y + b - 1).z - z) < edge_threshold_)
              nz += (*input_) (x + a - 1, y + b - 1).z * F (a, b);
            else
              nz += z * F (a, b);

        pc_out (x, y) = (*input_) (x, y);
        pc_out (x, y).z = nz / sum;

        if (last_pc_.width == input_->width && last_pc_.height == input_->height
            && std::abs (last_pc_ (x, y).z - pc_out (x, y).z) < edge_threshold_)
          pc_out (x, y).z = (1 - integral_factor_) * pc_out (x, y).z + integral_factor_ * last_pc_ (x, y).z;
      }

    last_pc_ = pc_out;
  }

#define PCL_INSTANTIATE_SmoothingFilter(T) template class cob_3d_mapping_filters::SmoothingFilter<T>;
#endif /* INTENSITY_FILTER_HPP_ */
