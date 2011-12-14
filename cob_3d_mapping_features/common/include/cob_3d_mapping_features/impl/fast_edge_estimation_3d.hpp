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

#ifndef __IMPL_FAST_EDGE_ESTIMATION_3D_H__
#define __IMPL_FAST_EDGE_ESTIMATION_3D_H__

#include "cob_3d_mapping_features/fast_edge_estimation_3d.h"
#include <cfloat>
#include "cob_3d_mapping_common/stop_watch.h"

template <typename PointInT, typename PointNT, typename PointOutT> void
cob_3d_mapping_features::FastEdgeEstimation3D<PointInT, PointNT, PointOutT>::isEdgePoint (
    const pcl::PointCloud<PointInT> &cloud,
    const PointInT &q_point,
    const std::vector<int> &indices,
    const Eigen::Vector3f &n,
    float &strength)
{
  if (indices.size () < 3)
  {
    strength = 0.0;
    return;
  }
  float nd_dot;
  Eigen::Vector3f delta;
  Eigen::Vector3f q = q_point.getVector3fMap();
  // Compute the angles between each neighboring point and the query point itself
  //int nn_ctr=0;//, nan_ctr=0;
  int b_ctr=0;
  float dist_threshold = 0.0015544 * pixel_search_radius_ * q.norm();

  for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
  {
    delta = cloud.points[*it].getVector3fMap() - q;
    nd_dot = n.dot (delta);
    if(fabs(nd_dot) > dist_threshold) //is border point
      b_ctr++;
  }
  //std::cout << dist_threshold << "\t" << indices.size() << "\t" << b_ctr << std::endl;
  strength = (float)b_ctr/indices.size();
  return;
}

template <typename PointInT, typename PointNT, typename PointOutT> void
cob_3d_mapping_features::FastEdgeEstimation3D<PointInT, PointNT, PointOutT>::computeFeature (
  PointCloudOut &output)
{
  std::vector<int> nn_indices;
  size_t idx = 0;
  Eigen::Vector3f normal;

  // Iterating over the entire index vector
  for (std::vector<int>::iterator it = indices_->begin(); it != indices_->end(); ++it)
  {
    if (isnan(normals_->points[*it].normal[0]))
      output.points[idx].strength=2;
    else
    {
      this->searchForNeighbors (*surface_, *it, nn_indices);

      normal[0] = normals_->points[*it].normal_x;
      normal[1] = normals_->points[*it].normal_y;
      normal[2] = normals_->points[*it].normal_z;
      // Estimate whether the point is lying on a boundary surface or not
      isEdgePoint (*surface_, input_->points[*it], nn_indices, 
		   normal, output.points[idx].strength);
    }
    idx++;
  }
}

#define PCL_INSTANTIATE_FastEdgeEstimation3D(PointInT,PointNT,PointOutT) template class PCL_EXPORTS cob_3d_mapping_features::FastEdgeEstimation3D<PointInT, PointNT, PointOutT>;

#endif    // __IMPL_FAST_EDGE_ESTIMATION_3D_H__
