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
 * \date Date of creation: 02/2012
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

#ifndef __IMPL_CURVATURE_CLASSIFIER_H__
#define __IMPL_CURVATURE_CLASSIFIER_H__

#include "cob_3d_mapping_features/curvature_classifier.h"


template <typename PointInT, typename PointOutT> void
cob_3d_mapping_features::CurvatureClassifier<PointInT,PointOutT>::classify(PointCloudOut &output)
{
  //std::cout << "da" << std::endl;
  if(!initCompute())
  {
    output.width = output.height = 0;
    output.points.clear ();
    std::cout << "error" << std::endl;
    return;
  }
  if (output.points.size() != input_->size())
  {
    output.points.resize(input_->size());
    output.height = input_->height;
    output.width = input_->width;
  }
  for (size_t i=0; i < indices_->size(); ++i)
  {
    float pc1 = std::fabs(input_->points[(*indices_)[i]].pc1);
    float pc2 = std::fabs(input_->points[(*indices_)[i]].pc2);
    if (output.points[i].label == I_BORDER) {continue;}
    if (pc1 < c_lower_)
    {
      output.points[i].label = I_PLANE;
    }
    else if (pc1 > c_upper_)
    {
      output.points[i].label = I_EDGE;
      /* // Uncomment for corner support:
      if (input_->points[(*indices_)[i]].pc1 < c_ratio_edge_corner_ * input_->points[(*indices_)[i]].pc2)
	output.points[i].label = I_CORNER;
      else
	output.points[i].label = I_EDGE;
      */
    }
    else if (pc1 < c_ratio_cylinder_sphere_ * pc2)
    {
      output.points[i].label = I_SPHERE;
    }
    else
    {
      output.points[i].label = I_CYL;
    }
  }
}

template <typename PointInT, typename PointOutT> void
cob_3d_mapping_features::CurvatureClassifier<PointInT,PointOutT>::classifyForSegmentation(PointCloudOut &output)
{
  //std::cout << "da" << std::endl;
  if(!initCompute())
  {
    output.width = output.height = 0;
    output.points.clear ();
    std::cout << "error" << std::endl;
    return;
  }
  if (output.points.size() != input_->size())
  {
    output.points.resize(input_->size());
    output.height = input_->height;
    output.width = input_->width;
  }
  for (size_t i=0; i < indices_->size(); ++i)
  {
    if (output.points[i].label == I_NAN) {continue;}
    if (output.points[i].label == I_BORDER) {output.points[i].label = I_EDGE; continue;}
    if (input_->points[(*indices_)[i]].pc1 > c_upper_)
    {
      output.points[i].label = I_EDGE;
    }
    else
    {
      output.points[i].label = I_UNDEF;
    }
  }
}

#define PCL_INSTANTIATE_CurvatureClassifier(T,OutT) template class PCL_EXPORTS cob_3d_mapping_features::CurvatureClassifier<T,OutT>;

#endif
