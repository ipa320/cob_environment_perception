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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
#ifndef SAV_GOL_SMOOTHING_FILTER_HPP_
#define SAV_GOL_SMOOTHING_FILTER_HPP_


#include <cob_3d_mapping_tools/sav_gol_smoothing_filter.h>

template <typename PointInT, typename PointOutT> bool
SavGolSmoothingFilter<PointInT,PointOutT>::smoothPoint(int index, float &z)
{
  int idx_x = index % input_->width;
  int idx_y = index / input_->width;
  int i = 0, miss = 0;

  float z_i;
  double sum = 0.0;
  float limit = 0.4 * size_ * size_;

  for (int y = idx_y - r_; y <= idx_y + r_; y++)
  {
    for (int x = idx_x - r_; x <= idx_x + r_; x++)
    {
      z_i = input_->at(x,y).z;
      if (pcl_isnan((float)z_i) || std::fabs(z_i-input_->points[index].z) > high_th_) 
      {
	z_i = input_->points[index].z;
	miss++;
	if (miss > limit) return false;
      }
      
      sum += coef_[i] * z_i;
      i++;
    }
  }
  //std::cout << "old: " << z << " new: " << sum << std::endl;
  z = sum;
  return true;
}

template <typename PointInT, typename PointOutT> void
SavGolSmoothingFilter<PointInT,PointOutT>::reconstruct(PointCloudOut &output, std::vector<size_t> &ignored_indices)
{
  output = *input_;
  int idx = 0;
  
  for (int y = r_+1; y < input_->height - r_; y++)
  {
    for (int x = r_+1; x < input_->width - r_; x++)
    {
      idx = y*input_->width + x;
      if (pcl_isnan(input_->at(x,y).z))
	continue;
      if ( !smoothPoint(idx, output.points[(size_t)idx].z) )
      {
	ignored_indices.push_back((size_t)idx);
      }
    }
  }
  return;
}

template <typename PointInT, typename PointOutT> void
SavGolSmoothingFilter<PointInT,PointOutT>::reconstruct2ndPass(
  std::vector<size_t> &indices, PointCloudOut &output)
{
  std::vector<size_t>::iterator it;

  for (it = indices.begin(); it != indices.end(); it++)
  {
    smoothPoint(*it, output.points[*it].z);
  }
  return;
}

#endif // SAV_GOL_SMOOTHING_FILTER_HPP_
