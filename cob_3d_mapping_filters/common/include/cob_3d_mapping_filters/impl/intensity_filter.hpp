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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_filters
 *
 * \author
 *  Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2011
 *
 * \brief
 * Description:
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
#ifndef INTENSITY_FILTER_HPP_
#define INTENSITY_FILTER_HPP_

//##################
//#### includes ####

// cob_3d_mapping_filters includes
#include "cob_3d_mapping_filters/intensity_filter.h"

template <typename PointT> void
cob_3d_mapping_filters::IntensityFilter<PointT>::applyFilter (PointCloud &pc_out)
{
	if(negative_) {
	   negativeApplyFilter(pc_out);
	   return;
   }
  // set the parameters for output poincloud (pc_out)
  pc_out.points.resize(input_->points.size());
  pc_out.header = input_->header;
  int nr_p = 0;

  //Go through all points and discard points with intensity value above filter limit
  for (unsigned int i = 0; i < input_->points.size(); i++)
  {
	  if (input_->points[i].intensity > intensity_min_threshold_ &&
		  input_->points[i].intensity < intensity_max_threshold_)
    pc_out.points[nr_p++] = input_->points[i];
  }

  //resize pc_out according to filtered points
  pc_out.width = nr_p;
  pc_out.height = 1;
  pc_out.points.resize(nr_p);
  pc_out.is_dense = true;
}

template <typename PointT> void
cob_3d_mapping_filters::IntensityFilter<PointT>::negativeApplyFilter (PointCloud &pc_out)
{
  // set the parameters for output poincloud (pc_out)
  pc_out.points.resize(input_->points.size());
  pc_out.header = input_->header;
  int nr_p = 0;

  //Go through all points and discard points with intensity value below filter limit
  for (unsigned int i = 0; i < input_->points.size(); i++)
  {
	  if (input_->points[i].intensity > intensity_max_threshold_ || input_->points[i].intensity < intensity_min_threshold_)
          pc_out.points[nr_p++] = input_->points[i];
  }

  //resize pc_out according to filtered points
  pc_out.width = nr_p;
  pc_out.height = 1;
  pc_out.points.resize(nr_p);
  pc_out.is_dense = true;

}

#define PCL_INSTANTIATE_IntensityFilter(T) template class cob_3d_mapping_filters::IntensityFilter<T>;
#endif /* INTENSITY_FILTER_HPP_ */
