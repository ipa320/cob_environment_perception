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

#ifndef __IMPL_ORGANIZED_CURVATURE_ESTIMATION_H__
#define __IMPL_ORGANIZED_CURVATURE_ESTIMATION_H__

#include <pcl/common/eigen.h>
#include "cob_3d_mapping_features/organized_curvature_estimation.h"

template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT> void
cob_3d_mapping_features::OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::computePointCurvatures (
  const NormalCloudIn &normals,
  const int index,
  const std::vector<int> &indices,
  float &pcx, float &pcy, float &pcz, float &pc1, float &pc2,
  int &label_out)
{
  std::vector<float> weights(indices.size(),1.0f);
  computePointCurvatures(normals, index, indices, weights, pcx, pcy, pcz, pc1, pc2, label_out);
}


template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT> void
cob_3d_mapping_features::OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::computePointCurvatures (
  const NormalCloudIn &normals,
  const int index,
  const std::vector<int> &indices,
  const std::vector<float> &sqr_distances,
  float &pcx, float &pcy, float &pcz, float &pc1, float &pc2,
  int &label_out)
{
  Eigen::Vector3f n_idx(normals.points[index].normal);
  Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - n_idx * n_idx.transpose(); // projection matrix

  std::vector<Eigen::Vector3f> normals_projected;
  normals_projected.reserve(n_points_);
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
  {
    PointNT const* n_i = &(normals.points[*it]);
    if ( pcl_isnan(n_i->normal[2]) ) continue;
    normals_projected.push_back( M * Eigen::Vector3f(n_i->normal) );
    centroid += normals_projected.at(normals_projected.size()-1);
  }

  if (normals_projected.size() <=1) return;
  float num_p_inv = 1.0f / normals_projected.size();
  centroid *= num_p_inv;

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  {
    std::vector<Eigen::Vector3f>::iterator n_it = normals_projected.begin();
    std::vector<float>::const_iterator d_it = sqr_distances.begin();
    for (; n_it != normals_projected.end(); ++n_it, ++d_it)
    {
      Eigen::Vector3f demean = *n_it - centroid;
      cov += 1.0f / sqrt(*d_it) * demean * demean.transpose();
    }
  }

  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  pcx = eigenvectors (0,2);
  pcy = eigenvectors (1,2);
  pcz = eigenvectors (2,2);
  num_p_inv *= surface_->points[index].z;
  pc1 = eigenvalues (2) * num_p_inv;
  pc2 = eigenvalues (1) * num_p_inv;
  //normals_->points[index].curvature = curvatures_->points[index].pc1;
  //std::cout << pc1 << " " << pc2 << std::endl;
  if (pc1 >= edge_curvature_threshold_ && label_out == 0)
    label_out = I_EDGE;
}


template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT> void
cob_3d_mapping_features::OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::computeFeature (PointCloudOut &output)
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

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  for (std::vector<int>::iterator it=indices_->begin(); it != indices_->end(); ++it)
  {
    if (pcl_isnan(surface_->points[*it].z) ||
	pcl_isnan(normals_->points[*it].normal[2]))
    {
      labels_->points[*it].label = I_NAN;
    }
    else if (this->searchForNeighborsInRange(*it, nn_indices, nn_distances) != -1)
    {
      computePointCurvatures(*normals_, *it, nn_indices, nn_distances,
			     output.points[*it].principal_curvature[0],
			     output.points[*it].principal_curvature[1],
			     output.points[*it].principal_curvature[2],
			     output.points[*it].pc1,
			     output.points[*it].pc2,
			     labels_->points[*it].label);
    }
    else
    {
      labels_->points[*it].label = I_NAN;
    }
  }
}

#define PCL_INSTANTIATE_OrganizedCurvatureEstimation(T,NT,LabelT,OutT) template class PCL_EXPORTS cob_3d_mapping_features::OrganizedCurvatureEstimation<T,NT,LabelT,OutT>;


#endif
