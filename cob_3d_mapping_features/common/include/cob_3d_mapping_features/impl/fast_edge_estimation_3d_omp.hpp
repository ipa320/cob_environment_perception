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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 10/2011
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

#ifndef __IMPL_FAST_EDGE_ESTIMATION_3D_OMP_H__
#define __IMPL_FAST_EDGE_ESTIMATION_3D_OMP_H__

#include "cob_3d_mapping_features/fast_edge_estimation_3d_omp.h"

template <typename PointInT, typename PointNT, typename PointOutT> void
cob_3d_mapping_features::FastEdgeEstimation3DOMP<PointInT, PointNT, PointOutT>::computeFeature (
  PointCloudOut &output)
{
  int threadsize = 1;

#pragma omp parallel for schedule (dynamic, threadsize)
  for (size_t i=0; i < indices_->size(); ++i)
  {
    Eigen::Vector3f normal;
    std::vector<int> nn_indices;
    if (isnan(normals_->points[(*indices_)[i]].normal[0]))
      output.points[(*indices_)[i]].strength=2;
    else
    {
      this->searchForNeighbors (*surface_, (*indices_)[i], nn_indices);

      normal[0] = normals_->points[(*indices_)[i]].normal_x;
      normal[1] = normals_->points[(*indices_)[i]].normal_y;
      normal[2] = normals_->points[(*indices_)[i]].normal_z;
      // Estimate whether the point is lying on a boundary surface or not
      isEdgePoint (*surface_, input_->points[(*indices_)[i]], nn_indices, 
		   normal, output.points[(*indices_)[i]].strength);
    }
  }
}

#define PCL_INSTANTIATE_FastEdgeEstimation3DOMP(PointInT,PointNT,PointOutT) template class PCL_EXPORTS cob_3d_mapping_features::FastEdgeEstimation3DOMP<PointInT, PointNT, PointOutT>;

#endif    // __IMPL_FAST_EDGE_ESTIMATION_3D_H__
