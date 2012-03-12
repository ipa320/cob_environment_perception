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
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 12/2011
 * ToDo:
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

#ifndef __IMPL_ORGANIZED_CURVATURE_ESTIMATION_OMP_H__
#define __IMPL_ORGANIZED_CURVATURE_ESTIMATION_OMP_H__

#include "cob_3d_mapping_features/organized_curvature_estimation_omp.h"

template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT> void
cob_3d_mapping_features::OrganizedCurvatureEstimationOMP<PointInT,PointNT,PointLabelT,PointOutT>::computeFeature (PointCloudOut &output)
{
  if (labels_->points.size() != input_->size())
  {
    labels_->points.resize(input_->size());
    labels_->height = input_->height;
    labels_->width = input_->width;
  }
  if (output.points.size() != input_->size())
  {
    output.points.resize(input_->size());
    output.height = input_->height;
    output.width = input_->width;
  }

  int threadsize = 1;

#pragma omp parallel for schedule (dynamic, threadsize)
  for (size_t i=0; i < indices_->size(); ++i)
  {
    std::vector<int> nn_indices;
    if (this->searchForNeighborsInRange(*surface_, (*indices_)[i], nn_indices) != -1)
    {
      computePointCurvatures(*normals_, (*indices_)[i], nn_indices,
			     output.points[(*indices_)[i]].principal_curvature[0],
			     output.points[(*indices_)[i]].principal_curvature[1],
			     output.points[(*indices_)[i]].principal_curvature[2],
			     output.points[(*indices_)[i]].pc1,
			     output.points[(*indices_)[i]].pc2,
			     labels_->points[(*indices_)[i]].label);
    }
    else
    {
      labels_->points[(*indices_)[i]].label = I_NAN;
    }
  }
}

#define PCL_INSTANTIATE_OrganizedCurvatureEstimationOMP(T,NT,LabelT,OutT) template class PCL_EXPORTS cob_3d_mapping_features::OrganizedCurvatureEstimationOMP<T,NT,LabelT,OutT>;

#endif
