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

#ifndef __IMPL_ORGANIZED_NORMAL_ESTIMATION_H__
#define __IMPL_ORGANIZED_NORMAL_ESTIMATION_H__


#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_features/organized_normal_estimation.h"
#include <pcl/common/eigen.h>

template <typename PointT, typename LabelT> void
cob_3d_mapping_features::OrganizedNormalEstimationHelper::computeSegmentNormal(
  Eigen::Vector3f& normal_out,
  int index,
  boost::shared_ptr<const pcl::PointCloud<PointT> > surface,
  boost::shared_ptr<const pcl::PointCloud<LabelT> > labels,
  int r, int steps)
{
  const int w = surface->width, s = surface->height * surface->width;
  const int l_idx = labels->points[index].label;

  // compute mask boundary constrains first,
  const int w_rem = index%w;
  const int x_max = std::min(2*r, r + w - w_rem - 1); // max offset at each line
  const int y_min = std::max(index - r*w - r, w_rem - r);
  const int y_max = std::min(index + r*w - r, s - (w - w_rem) - r);
  Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
  int point_count = 0;
  for (int y = y_min; y <= y_max; y += steps*w) // y: beginning of each line
  {
    for (int idx = y; idx < y + x_max; idx+=steps)
    {
      if (labels->points[idx].label != l_idx) { idx = idx - steps + 1; continue; }
      PointT const* p_idx = &(surface->points[idx]);
      if (pcl_isnan(p_idx->x)) { idx = idx - steps + 1; continue; }
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
    if (point_count == 0) { y = y - steps*w + w; }
  }
  /*if (point_count <= 1)
  {
    std::cout << "Still to less valid points" << std::endl;
    //normal_out(0) = normal_out(1) = normal_out(2) = std::numeric_limits<float>::quiet_NaN();
    //return;
    }*/
  if (accu[0] == 0 && accu[3] == 0 && accu[5] == 0)
  {
    std::cout <<"[NormalEstimation]: null normal!" << std::endl;
    normal_out(0) = normal_out(1) = normal_out(2) = std::numeric_limits<float>::quiet_NaN();
    return;
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
  if ( surface->points[index].getVector3fMap().dot(eigenvectors.col(0)) > 0)
    normal_out = eigenvectors.col(0) * (-1);
  else
    normal_out = eigenvectors.col(0);
}

template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_3d_mapping_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::recomputeSegmentNormal (
  PointCloudInConstPtr cloud_in,
  LabelCloudOutConstPtr label_in,
  int index,
  float& n_x,
  float& n_y,
  float& n_z)
{
  Eigen::Vector3f p = cloud_in->points[index].getVector3fMap();
  if (pcl_isnan(p(2)))
  {
    n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
    return;
  }


  int idx, max_gab, gab, init_gab, n_normals = 0;
  int idx_x = index % cloud_in->width;
  int idx_y = index / cloud_in->width;
  int l_idx = label_in->points[index].label;

  bool has_prev_point;

  Eigen::Vector3f p_curr;
  Eigen::Vector3f p_prev(0,0,0);
  Eigen::Vector3f p_first(0,0,0);
  Eigen::Vector3f n_idx(0,0,0);

  std::vector<std::vector<int> >::iterator it_c; // circle iterator
  std::vector<int>::iterator it_ci; // points in circle iterator

  // check where query point is and use out-of-image validation for neighbors or not
  if (idx_y >= pixel_search_radius_ && idx_y < (int)cloud_in->height - pixel_search_radius_ &&
      idx_x >= pixel_search_radius_ && idx_x < (int)cloud_in->width - pixel_search_radius_)
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c) // iterate circles
    {
      has_prev_point = false; init_gab = gab = 0; max_gab = 0.25 * (*it_c).size(); // reset loop

      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
      {
        idx = index + *it_ci;
        //std::cout << idx % cloud_in->width << " / " << idx / cloud_in->width << std::endl;
        Eigen::Vector3f p_i = cloud_in->points[idx].getVector3fMap();
        if ( pcl_isnan(p_i(2)) || label_in->points[idx].label != l_idx) { ++gab; continue; } // count as gab point
        if ( gab <= max_gab && has_prev_point ) // check gab is small enough and a previous point exists
        {
          p_curr = p_i - p;
          n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
          ++n_normals;
          p_prev = p_curr;
        }
        else // current is first point in circle or just after a gab
        {
          p_prev = p_i - p;
          if (!has_prev_point)
          {
            p_first = p_prev; // remember the first valid point in circle
            init_gab = gab; // save initial gab size
            has_prev_point = true;
          }
        }
        gab = 0; // found valid point, reset gab counter
      }

      // close current circle (last and first point) if gab is small enough
      if (gab + init_gab <= max_gab)
      {
        // compute normal of p_first and p_prev
        n_idx += (p_prev.cross(p_first)).normalized();
        ++n_normals;
      }
    } // end loop of circles
  }
  else
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c) // iterate circles
    {
      has_prev_point = false; init_gab = gab = 0; max_gab = 0.25 * (*it_c).size(); // reset loop

      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
      {
        idx = index + *it_ci;
        //std::cout << idx % cloud_in->width << " / " << idx / cloud_in->width << std::endl;
        if ( idx < 0 || idx >= (int)cloud_in->points.size() ) { continue; }
        int v = idx * inv_width_; // calculate y coordinate in image, // check left, right border
        if ( v < 0 || v >= (int)cloud_in->height ) { continue; }
        Eigen::Vector3f p_i = cloud_in->points[idx].getVector3fMap();
        if ( pcl_isnan(p_i(2)) || label_in->points[idx].label != l_idx) { ++gab; continue; } // count as gab point
        if ( gab <= max_gab && has_prev_point ) // check gab is small enough and a previous point exists
        {
          p_curr = p_i - p;
          n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
          ++n_normals;
          p_prev = p_curr;
        }
        else // current is first point in circle or just after a gab
        {
          p_prev = p_i - p;
          if (!has_prev_point)
          {
            p_first = p_prev; // remember the first valid point in circle
            init_gab = gab; // save initial gab size
            has_prev_point = true;
          }
        }
        gab = 0; // found valid point, reset gab counter
      }

      // close current circle (last and first point) if gab is small enough
      if (gab + init_gab <= max_gab)
      {
        // compute normal of p_first and p_prev
        n_idx += (p_prev.cross(p_first)).normalized();
        ++n_normals;
      }
    } // end loop of circles
  }

  n_idx /= (float)n_normals;
  n_idx = n_idx.normalized();
  n_x = n_idx(0);
  n_y = n_idx(1);
  n_z = n_idx(2);
}


template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_3d_mapping_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::computePointNormal (
  const PointCloudIn &cloud,
  int index,
  float &n_x,
  float &n_y,
  float &n_z,
  int &label_out)
{
  Eigen::Vector3f p = cloud.points[index].getVector3fMap();
  if (pcl_isnan(p(2)))
  {
    n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
    label_out = I_NAN;
    return;
  }

  int idx, max_gab, gab, init_gab, n_normals = 0;
  int idx_x = index % input_->width;
  int idx_y = index * inv_width_;

  float distance_threshold = skip_distant_point_threshold_ * 0.003 * p(2) * p(2);

  bool has_prev_point;

  std::vector<int> range_border_counter(mask_.size(), 0);
  Eigen::Vector3f p_curr;
  Eigen::Vector3f p_prev(0,0,0);
  Eigen::Vector3f p_first(0,0,0);
  Eigen::Vector3f n_idx(0,0,0);

  std::vector<std::vector<int> >::iterator it_c; // circle iterator
  std::vector<int>::iterator it_ci; // points in circle iterator
  std::vector<int>::iterator it_rbc = range_border_counter.begin();

  // check where query point is and use out-of-image validation for neighbors or not
  if (idx_y >= pixel_search_radius_ && idx_y < (int)cloud.height - pixel_search_radius_ &&
      idx_x >= pixel_search_radius_ && idx_x < (int)cloud.width - pixel_search_radius_)
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c, ++it_rbc) // iterate circles
    {
      has_prev_point = false; init_gab = gab = 0; max_gab = 0.25 * (*it_c).size(); // reset loop

      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
      {
        idx = index + *it_ci;
        Eigen::Vector3f p_i = cloud.points[idx].getVector3fMap();
        if ( pcl_isnan(p_i(2)) )                        { ++gab; continue; }               // count as gab point
        if ( fabs(p_i(2) - p(2)) > distance_threshold ) { ++gab; ++(*it_rbc); continue; }  // count as gab point
        if ( gab <= max_gab && has_prev_point ) // check gab is small enough and a previous point exists
        {
          p_curr = p_i - p;
          n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
          ++n_normals;
          p_prev = p_curr;
        }
        else // current is first point in circle or just after a gab
        {
          p_prev = p_i - p;
          if (!has_prev_point)
          {
            p_first = p_prev; // remember the first valid point in circle
            init_gab = gab; // save initial gab size
            has_prev_point = true;
          }
        }
        gab = 0; // found valid point, reset gab counter
      }

      // close current circle (last and first point) if gab is small enough
      if (gab + init_gab <= max_gab)
      {
        // compute normal of p_first and p_prev
        n_idx += (p_prev.cross(p_first)).normalized();
        ++n_normals;
      }
    } // end loop of circles
  }
  else
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c, ++it_rbc) // iterate circles
    {
      has_prev_point = false; gab = 0; max_gab = 0.25 * (*it_c).size(); // reset circle loop

      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
      {
        idx = index + *it_ci;
        // check top and bottom image border
        if ( idx < 0 || idx >= (int)cloud.points.size() ) { ++gab; continue; } // count as gab point
        int v = idx * inv_width_; // calculate y coordinate in image, // check left, right border
        if ( v < 0 || v >= (int)cloud.height || pcl_isnan(cloud.points[idx].z)) { ++gab; continue; } // count as gab point
        Eigen::Vector3f p_i = cloud.points[idx].getVector3fMap();
        if ( fabs(p_i(2) - p(2)) > distance_threshold ) { ++gab; ++(*it_rbc); continue; }  // count as gab point
        if ( gab <= max_gab && has_prev_point) // check gab is small enough and a previous point exists
        {
          p_curr = p_i - p;
          n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
          ++n_normals;
          p_prev = p_curr;
        }
        else // current is first point in circle or just after a gab
        {
          p_prev = p_i - p;
          if (!has_prev_point)
          {
            p_first = p_prev; // remember the first valid point in circle
            init_gab = gab; // save initial gab size
            has_prev_point = true;
          }
        }
        gab = 0; // found valid point, reset gab counter
      }

      // close current circle (last and first point) if gab is small enough
      if ( gab + init_gab <= max_gab)
      {
        // compute normal of p_first and p_prev
        n_idx += (p_prev.cross(p_first)).normalized();
        ++n_normals;
      }
    } // end loop of circles
  }

  //if (range_border_counter[mask_.size()-1] > 0) label_out = I_BORDER;

  n_idx /= (float)n_normals;
  n_idx = n_idx.normalized();
  n_x = n_idx(0);
  n_y = n_idx(1);
  n_z = n_idx(2);
}

template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_3d_mapping_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::computeFeature (PointCloudOut &output)
{
  if (labels_->points.size() != input_->size())
  {
    labels_->points.resize(input_->size());
    labels_->height = input_->height;
    labels_->width = input_->width;
  }

  for (std::vector<int>::iterator it=indices_->begin(); it != indices_->end(); ++it)
  {
    labels_->points[*it].label = I_UNDEF;
    computePointNormal(*surface_, *it,
                       output.points[*it].normal[0],
                       output.points[*it].normal[1],
                       output.points[*it].normal[2],
                       labels_->points[*it].label);
  }
}

#define PCL_INSTANTIATE_OrganizedNormalEstimation(T,OutT,LabelT) template class PCL_EXPORTS cob_3d_mapping_features::OrganizedNormalEstimation<T,OutT,LabelT>;

#endif
