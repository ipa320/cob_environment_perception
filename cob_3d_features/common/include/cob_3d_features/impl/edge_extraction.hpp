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
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2011
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

#ifndef __IMPL_EDGE_EXTRACTION_H__
#define __IMPL_EDGE_EXTRACTION_H__

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_features/edge_extraction.h"

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeExtraction<PointInT,PointOutT>::extractEdges(
  PointCloudOut &output)
{
  output.height = input_3d_->height;
  output.width = input_3d_->width;
  output.points.resize(output.height * output.width);

  float thresh = threshold_;// * 2;

  for (size_t i=0; i < output.size(); i++)
  {
    if (input_3d_->points[i].strength > 1.0)
    {
      output.points[i].label = I_NAN;
    }
    //TODO: apply weight factor
    else if (input_3d_->points[i].strength > thresh*2 || (input_2d_->points[i].strength+0.1) > thresh)
    {
      // add normalized 2D and 3D strength values and apply threshold
      output.points[i].label = I_EDGE;
    }
    else
    {
      output.points[i].label = I_UNDEF;
    }
  }
}

#define PCL_INSTANTIATE_EdgeExtraction(T,OutT) template class PCL_EXPORTS cob_3d_features::EdgeExtraction<T,OutT>;

#endif
