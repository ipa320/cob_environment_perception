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
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
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

#ifndef __IMPL_FAST_SEGMENTATION_HPP__
#define __IMPL_FAST_SEGMENTATION_HPP__

#include <set>

#include <pcl/common/eigen.h>
#include <Eigen/LU>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_segmentation/fast_segmentation.h"

template <typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::FastSegmentation<PointT,PointNT,PointLabelT>::createSeedPoints()
{
  int n = labels_->width * labels_->height;
  p_seeds_.clear();
  seeds_.clear();
  switch (seed_method_)
  {
  case SEED_RANDOM:
  {
    p_seeds_.push_back(seeds_.insert(seeds_.end(), 0));
    for(unsigned int i = 1; i<n; ++i)
    {
      if(surface_->points[i].z != surface_->points[i].z)
        p_seeds_.push_back(p_seeds_[rand()%i]);
      else
        p_seeds_.push_back(seeds_.insert(p_seeds_[rand()%i],i));
    }
    p_seeds_[0] = seeds_.insert(p_seeds_[rand()%n],0);
    seeds_.pop_back();
    break;
  }

  case SEED_LINEAR:
  {
    for(unsigned int i = 0; i<n; ++i)
    {
      if(surface_->points[i].z != surface_->points[i].z)
        p_seeds_.push_back(seeds_.end());
      else
        p_seeds_.push_back(seeds_.insert(seeds_.end(),i));
    }
    break;
  }
}

template <typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::FastSegmentation<PointT,PointNT,PointLabelT>::addIfIsValid(
  int u, int v, int idx, int idx_prev, float dist_th, float p_z, Eigen::Vector3f& n,
  SegmentationQueue& seg_queue, ClusterPtr c)
{
  if (p_z < 1.2)
  {
    if (fabs(p_z - surface_->points[idx].z) > (dist_th + 0.01f)) return;
  }
  else if (fabs(p_z - surface_->points[idx].z) > dist_th) return;
  int* p_label = &(labels_->points[idx].label);
  if (*p_label == I_UNDEF)
  {
    float dot_value = fabs( n.dot(normals_->points[idx].getNormalVector3fMap()) );
    if ( (int)c->size() < min_cluster_size_ || dot_value > min_dot_normals_)
    {
      *p_label = c->id();
      seg_queue.push( SegmentationCandidate::Ptr(new SegmentationCandidate(u, v, dot_value)) );
    }
  }
  else if (*p_label > I_EDGE && c->id() != *p_label)
  {
    graph_->edges()->updateProperties(graph_->connect(c->id(), *p_label), c->id(), idx_prev, *p_label, idx);
  }
}

template <typename PointT, typename PointNT, typename PointLabelT> bool
cob_3d_segmentation::FastSegmentation::compute()
{
  int w = labels_->width, h = labels_->height;
  clusters_->clear();
  while(seeds_.size() != 0)
  {
    unsigned int i = seeds_.front();
    seeds_.pop_front();
    
  }
}

