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
 * ROS package name: cob_3d_segmentation
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
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

#ifndef __IMPL_CLUSTER_CLASSIFIER_HPP__
#define __IMPL_CLUSTER_CLASSIFIER_HPP__

#include "cob_3d_segmentation/cluster_classifier.h"
#include <cob_3d_mapping_features/organized_normal_estimation.h>

#include <pcl/common/eigen.h>

#include <boost/tuple/tuple.hpp>

template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> void
cob_3d_segmentation::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::classifyOld()
{
  ClusterPtr c_it, c_end;
  for ( boost::tie(c_it,c_end) = clusters_->getClusters(); c_it != c_end; ++c_it)
  {
    if ( c_it->size() < 5 ) continue;
    if ( c_it->type == I_EDGE || c_it->type == I_NAN || c_it->type == I_BORDER) continue;

    clusters_->computeClusterComponents(c_it);

    if (!c_it->is_save_plane)
    {
      
      recomputeClusterNormals(c_it);
      clusters_->computeCurvature(c_it);
      if (c_it->max_curvature < 0.02)
	c_it->type = I_PLANE;
      else if (c_it->max_curvature < 9.0 * c_it->min_curvature)
	c_it->type = I_SPHERE;
      else
	c_it->type = I_CYL;
      
      //c_it->type = I_CORNER;
    }
    else c_it->type = I_PLANE;
  }
}

template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> void
cob_3d_segmentation::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::classify()
{
  ClusterPtr c_it, c_end;
  float pc_min, pc_max;
  test.clear();
  for ( boost::tie(c_it,c_end) = clusters_->getClusters(); c_it != c_end; ++c_it)
  {
    if ( c_it->size() < 5 ) continue;
    if ( c_it->type == I_EDGE || c_it->type == I_NAN || c_it->type == I_BORDER) continue;

    clusters_->computeClusterComponents(c_it);

    if (!c_it->is_save_plane)
    {
      int w_size = std::min( std::floor(sqrt(c_it->size() / 16.0f)) + 5, 30.0 );
      int steps = std::floor(w_size / 5);
      int w_size_n = std::min( std::floor(sqrt(c_it->size() / 16.0f)) + 8, 30.0 );
      int steps_n = std::floor(w_size / 5);
      recomputeClusterNormals(c_it, w_size_n, steps_n);
      std::vector<int> geometry(NUM_LABELS, 0);
      int valid_points = 0;
      for (typename ClusterType::iterator idx=c_it->begin(); idx != c_it->end(); ++idx)
      {
	if (!computeClusterPointCurvature(*idx, w_size, steps, pc_min, pc_max)) 
	{
	  test.push_back(*idx);
	  continue;
	}
	++valid_points;
	if (pc_max < 0.01) { ++geometry[I_PLANE]; test_plane.push_back(*idx); }
	else if(pc_max > 9.0 * pc_min) { ++geometry[I_CYL]; test_cyl.push_back(*idx); }
	else { ++geometry[I_SPHERE]; test_sph.push_back(*idx); }
      }

      int max = 0; size_t max_idx = 0;
      for (size_t i=0; i<geometry.size(); ++i)
	if ( (max=std::max(max,geometry[i])) == geometry[i] ) max_idx=i;

      if (max == 0) { c_it->type = I_EDGE; }
      else 
      { 
	c_it->type = max_idx;
	c_it->type_probability = static_cast<float>(max) / static_cast<float>(valid_points);
	//std::cout << "Label: " << max_idx << " : #" << c_it->type_probability << std::endl;
	if (c_it->type == I_CYL && c_it->type_probability < 0.8) { c_it->type = I_SPHERE; }
      }
    }
    else c_it->type = I_PLANE;
  }
}

template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> void
cob_3d_segmentation::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::recomputeClusterNormals(ClusterPtr c)
{
  int w_size = std::min( std::floor(sqrt(c->size() / 16.0f)) + 5, 30.0);
  int steps = std::floor(w_size / 5);
  recomputeClusterNormals(c, w_size, steps);
}

template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> void
cob_3d_segmentation::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::recomputeClusterNormals(
  ClusterPtr c,
  int w_size,
  int steps)
{
  clusters_->clearOrientation(c);
  for (typename ClusterType::iterator idx = c->begin(); idx != c->end(); ++ idx)
  {
    Eigen::Vector3f new_normal;
    cob_3d_mapping_features::OrganizedNormalEstimationHelper::computeSegmentNormal<PointT,LabelT>(
      new_normal, *idx, surface_, labels_, w_size, steps);
    normals_->points[*idx].getNormalVector3fMap() = new_normal;
    clusters_->updateNormal(c, new_normal);
  }
}

template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> bool
cob_3d_segmentation::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::computeClusterPointCurvature(
  int index, int r, int steps, float& pc_min, float& pc_max)
{
  const int w = surface_->width, s = surface_->height * surface_->width;
  const int l_idx = labels_->points[index].label;

  // compute mask boundary constrains first, 
  const int w_rem = index%w;
  const int x_max = std::min(2*r, r + w - w_rem - 1); // max offset at each line
  const int y_min = std::max(index - r*w - r, w_rem - r);
  const int y_max = std::min(index + r*w - r, s - (w - w_rem) - r);

  const Eigen::Vector3f n_idx(normals_->points[index].normal);
  const Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - n_idx * n_idx.transpose();

  std::vector<Eigen::Vector3f> normals_projected;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

  for (int y = y_min; y <= y_max; y += steps*w) // y: beginning of each line
  {
    for (int idx = y; idx < y + x_max; idx+=steps)
    {
      if ( labels_->points[idx].label != l_idx ) { continue; }
      NormalT const* n_i = &(normals_->points[idx]);
      if ( pcl_isnan(n_i->normal[2]) ) continue;
      normals_projected.push_back( M * n_i->getNormalVector3fMap() );
      centroid += normals_projected.back();
    }
  }
  if (normals_projected.size() < 0.5 * pow(ceil(static_cast<float>(2*r+1)/static_cast<float>(steps)), 2)) { return false; }
  float num_p_inv = 1.0f / normals_projected.size();
  centroid *= num_p_inv;
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (std::vector<Eigen::Vector3f>::iterator n_it = normals_projected.begin(); n_it != normals_projected.end(); ++n_it)
  {
    Eigen::Vector3f demean = *n_it - centroid;
    cov += demean * demean.transpose();
  }
  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  pc_max = eigenvalues(2) * num_p_inv;
  pc_min = eigenvalues(1) * num_p_inv;
  return true;
}

/*
template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> void
cob_3d_segmentation::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::computeClusterPointCurvature(ClusterPtr c)
{

  for (typename ClusterType::iterator idx=c->begin(); idx != c->end(); ++idx)
  {

   
    if (eigenvalues(2) < 0.01) ++geometry[I_PLANE];
    else if(eigenvalues(2) < 0.11)
    {
      if(eigenvalues(2) < 7.0 * eigenvalues(1)) ++geometry[I_SPHERE];
      else ++geometry[I_CYL];
    }
    else ++geometry[I_UNDEF];
  }
  int max = 0; size_t max_idx = 0;
  for (size_t i=0; i<geometry.size(); ++i)
    if ( (max=std::max(max,geometry[i])) == geometry[i] ) max_idx=i;
  c->type = max_idx;
}
*/

#endif
