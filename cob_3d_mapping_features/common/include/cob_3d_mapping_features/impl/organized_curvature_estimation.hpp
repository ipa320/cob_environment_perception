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
  std::vector<int> weights(indices.size(),1);
  computePointCurvatures(normals, index, weights, pcx, pcy, pcz, pc1, pc2, label_out);
}

template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT> void
cob_3d_mapping_features::OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::computePointCurvatures (
  const NormalCloudIn &normals,
  const int index,
  const std::vector<int> &indices,
  const std::vector<int> &distance_weights,
  float &pcx, float &pcy, float &pcz, float &pc1, float &pc2,
  int &label_out)
{
  if (pcl_isnan(normals.points[index].normal[2])) return;
  Eigen::Vector3f norm(normals.points[index].normal[0],
		       normals.points[index].normal[1],
		       normals.points[index].normal[2]);


  Eigen::Matrix3f cov, eigenvectors;
  Eigen::Vector3f centroid, demean, eigenvalues;
  centroid.setZero();
  Eigen::Matrix3f I = Eigen::Matrix3f::Identity ();
  Eigen::Matrix3f M = I - norm * norm.transpose(); // projection matrix
  std::vector<Eigen::Vector3f> normals_projected;
  std::vector<float> weights;
  float weight;

  for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
  {
    if (pcl_isnan(normals.points[*it].normal[2])) continue;
    norm[0] = normals.points[*it].normal[0];
    norm[1] = normals.points[*it].normal[1];
    norm[2] = normals.points[*it].normal[2];

    normals_projected.push_back( M * norm );
    centroid += normals_projected.at(normals_projected.size()-1);

    /*
    std::cout << *it << ": ";
    std::cout << normals.points[index].normal[0] << " "
	      << normals.points[index].normal[1] << " " 
	      << normals.points[index].normal[2] << "  |  ";
    std::cout << surface_->points[index].x << " "
	      << surface_->points[index].y << " " 
	      << surface_->points[index].z << std::endl;
    */
    weight = 
      pow(surface_->points[*it].x - surface_->points[index].x,2) +
      pow(surface_->points[*it].y - surface_->points[index].y,2) +
      pow(surface_->points[*it].z - surface_->points[index].z,2);

    weights.push_back(weight);
  }

  if (normals_projected.size() <=1) return;
  float num_p_inv = 1.0f / normals_projected.size();
  centroid *= num_p_inv;
  cov.setZero();

  for (size_t i=0; i<normals_projected.size(); ++i)
  {
    demean = normals_projected[i] - centroid;
    //cov += 1.0f / (distance_weights[i] * distance_weights[i]) * demean * demean.transpose();
    cov += 1.0f / (sqrt(weights[i])) * demean * demean.transpose();
  }

  pcl::eigen33(cov, eigenvectors, eigenvalues);
  pcx = eigenvectors (0,2);
  pcy = eigenvectors (1,2);
  pcz = eigenvectors (2,2);
  pc1 = eigenvalues (2) * num_p_inv * surface_->points[index].z;
  pc2 = eigenvalues (1) * num_p_inv * surface_->points[index].z;
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
  std::vector<int> nn_distances;

  for (std::vector<int>::iterator it=indices_->begin(); it != indices_->end(); ++it)
  {
    if (this->searchForNeighborsInRange(*surface_, *it, nn_indices, nn_distances) != -1)
    {
      //for (size_t i=0; i<nn_distances.size(); i++) std::cout << nn_distances[i] << "  ";
      //std::cout << std::endl;
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
