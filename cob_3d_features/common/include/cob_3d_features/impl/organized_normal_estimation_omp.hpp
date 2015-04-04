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
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
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

#ifndef __IMPL_ORGANIZED_NORMAL_ESTIMATION_OMP_H__
#define __IMPL_ORGANIZED_NORMAL_ESTIMATION_OMP_H__

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_features/organized_normal_estimation_omp.h"

template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_3d_features::OrganizedNormalEstimationOMP<PointInT,PointOutT,LabelOutT>::computeFeature (PointCloudOut &output)
{
  if (labels_->points.size() != input_->size())
  {
    labels_->points.resize(input_->size());
    labels_->height = input_->height;
    labels_->width = input_->width;
  }

  int threadsize = 1;

#pragma omp parallel for schedule (dynamic, threadsize)
  for (size_t i=0; i < indices_->size(); ++i)
  {
    labels_->points[(*indices_)[i]].label = I_UNDEF;
    this->computePointNormal(*surface_, (*indices_)[i],
		       output.points[(*indices_)[i]].normal[0],
		       output.points[(*indices_)[i]].normal[1],
		       output.points[(*indices_)[i]].normal[2],
		       labels_->points[(*indices_)[i]].label);
  }
}

#define PCL_INSTANTIATE_OrganizedNormalEstimationOMP(T,OutT,LabelT) template class PCL_EXPORTS cob_3d_features::OrganizedNormalEstimationOMP<T,OutT,LabelT>;

#endif

