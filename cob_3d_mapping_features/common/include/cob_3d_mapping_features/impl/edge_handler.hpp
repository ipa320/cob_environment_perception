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

#ifndef __IMPL_EDGE_HANDLER_HPP__
#define __IMPL_EDGE_HANDLER_HPP__

#include "cob_3d_mapping_features/edge_handler.h"
#include <pcl/common/eigen.h>

template<typename LabelT, typename PointT> void
cob_3d_mapping_features::BoundaryPointsEdgeHandler<LabelT,PointT>::erase(EdgePtr e)
{
  // TODO: delete boundary points
  EdgeHandlerBase<BoundaryPointsEdge>::erase(e);
}

template<typename LabelT, typename PointT> void
cob_3d_mapping_features::BoundaryPointsEdgeHandler<LabelT,PointT>::merge(EdgePtr source, EdgePtr target)
{
  /* Assume edge_src connects cluster "a" and "b", and edge_trg connects cluster "a" and "c":
   *
   * +-----------+
   * |     a     |   src_a -> trg_a (combine boundary points on side "a") 
   * +-----+-----+
   * |  b  |  c  |   src_b -> trg_c (move boundary point on side "b" to side "c")
   * +-----------+
   */

  std::map<int,std::list<int> >::iterator src_pair_1st = source->boundary_pairs.begin(); // pairs of first src cluster
  std::map<int,std::list<int> >::iterator src_pair_2nd = ++source->boundary_pairs.begin(); // pairs of second src cluster
  std::map<int,std::list<int> >::iterator trg_pair_1st = target->boundary_pairs.begin(); // pairs of first trg cluster
  std::map<int,std::list<int> >::iterator trg_pair_2nd = ++target->boundary_pairs.begin(); // pairs of second trg cluster

  // bring pointers in correct order ( src_1st will be appended to trg_1st and src_2nd to trg_2nd )
  if (src_pair_1st->first == trg_pair_2nd->first)
  { 
    trg_pair_1st = trg_pair_2nd; 
    trg_pair_2nd = target->boundary_pairs.begin();
  }
  else if (src_pair_2nd->first == trg_pair_1st->first) 
  {
    src_pair_1st = src_pair_2nd;
    src_pair_2nd = source->boundary_pairs.begin();
  }
  /* this is case shouldn't happen: (maybe throw exception or something)
     else if (src_pair_1st->first != trg_pair_1st->first && src_pair_2nd->first != trg_pair_2nd->first) 
     { 
     // no mutal clusters
     }
  */
  trg_pair_1st->second.insert(trg_pair_1st->second.begin(), src_pair_1st->second.begin(), src_pair_1st->second.end());
  trg_pair_2nd->second.insert(trg_pair_2nd->second.begin(), src_pair_2nd->second.begin(), src_pair_2nd->second.end());
  erase(source);
}

template<typename LabelT, typename PointT> void
cob_3d_mapping_features::BoundaryPointsEdgeHandler<LabelT,PointT>::computeBoundaryPointProperties(
  const int r,
  const int index,
  BoundaryPoint& bp)
{
  const int w = surface_->width, h = surface_->height, s = surface_->height * surface_->width;
  const int l_idx = labels_->points[index].label;

  // compute mask boundary constrains first, 
  const int w_rem = index%w;
  const int x_max = std::min(2*r, r + w - w_rem - 1); // max offset at each line
  const int y_min = std::max(index - r*w - r, w_rem - r);
  const int y_max = std::min(index + r*w - r, s - (w - w_rem) - r);
  Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
  int point_count = 0;
  for (int y = y_min; y <= y_max; y += w) // y: beginning of each line
  {
    for (int idx = y; idx < y + x_max; ++idx)
    {
      if (labels_->points[idx].label != l_idx) continue;
      PointT const* p_idx = &(surface_->points[idx]);
      if (pcl_isnan(p_idx->x)) continue;
      accu[0] += p_idx->x * p_idx->x;
      accu[1] += p_idx->x * p_idx->y;
      accu[2] += p_idx->x * p_idx->z;
      accu[3] += p_idx->y * p_idx->y;
      accu[4] += p_idx->y * p_idx->z;
      accu[5] += p_idx->z * p_idx->z;
      accu[6] += p_idx->x;
      accu[7] += p_idx->y;
      accu[8] += p_idx->z;
      ++point_count;
    }
  }
  accu /= static_cast<float>(point_count);
    
  Eigen::Matrix3f cov;
  cov.coeffRef(0) =                   accu(0) - accu(6) * accu(6);
  cov.coeffRef(1) = cov.coeffRef(3) = accu(1) - accu(6) * accu(7);
  cov.coeffRef(2) = cov.coeffRef(6) = accu(2) - accu(6) * accu(8);
  cov.coeffRef(4) =                   accu(3) - accu(7) * accu(7);
  cov.coeffRef(5) = cov.coeffRef(7) = accu(4) - accu(7) * accu(8);
  cov.coeffRef(8) =                   accu(5) - accu(8) * accu(8);
  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  if ( surface_->points[index].getVector3fMap().dot(eigenvectors.col(0)) > 0)
    bp.normal = eigenvectors.col(0) * (-1);
  else
    bp.normal = eigenvectors.col(0);
}

#endif
