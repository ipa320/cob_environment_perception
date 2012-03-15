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


// TODO: remove the storage of normals to increase speed (std::vector<Eigen::Vector3f> normals)


template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_3d_mapping_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::computePointNormal (
  const PointCloudIn &cloud,
  int index,
  float &n_x,
  float &n_y,
  float &n_z,
  int &label_out)
{
  PointInT p = cloud.points[index];

  if (pcl_isnan(p.z))
  {
    n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
    label_out = I_NAN;
    return;
  }

  int idx, max_gab, gab, init_gab;
  int idx_x = index % input_->width;
  int idx_y = index * inv_width_;

  float distance_threshold = distance_threshold_modifier_ * 0.003 * p.z * p.z;

  bool has_prev_point;

  std::vector<int> range_border_counter(mask_.size(), 0);
  std::vector<Eigen::Vector3f> normals;
  Eigen::Vector3f ni, p_curr;
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
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++, it_rbc++) // iterate circles 
    {
      has_prev_point = false; init_gab = gab = 0; max_gab = 0.25 * (*it_c).size(); // reset loop
    
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
	idx = index + *it_ci;
	if ( pcl_isnan(cloud.points[idx].z) )
	{
	  gab++; // count as gab point
	  continue;	  
	}
	float d = fabs(cloud.points[idx].z - p.z); 
	if ( d > distance_threshold )
	{
	  gab++; // count as gab point
	  //if ( d > 3.0*distance_threshold )
	  if ( d > distance_threshold )
	    *it_rbc += 1; // increment range border counter of current circle
	  continue;
	}
	if ( gab <= max_gab && has_prev_point) // check gab is small enough and a previous point exists
	{
	  p_curr = cloud.points[idx].getVector3fMap() - p.getVector3fMap();
	  ni = p_prev.cross(p_curr); // compute normal of p_prev and p_curr
	  ni = ni.normalized();
	  n_idx += ni; // add to mean normal
	  normals.push_back(ni);
	  p_prev = p_curr;
	}
	else // current is first point in circle or just after a gab
	{
	  p_prev = cloud.points[idx].getVector3fMap() - p.getVector3fMap();
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
	ni = p_prev.cross(p_first);
	ni = ni.normalized();
	n_idx += ni;
	normals.push_back(ni);
      }
    } // end loop of circles
  }
  else
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++, it_rbc++) // iterate circles 
    {
      has_prev_point = false; gab = 0; max_gab = 0.25 * (*it_c).size(); // reset circle loop
    
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
	idx = index + *it_ci;
	if ( idx < 0 || idx >= (int)cloud.points.size() ) // check top and bottom image border
	{
	  gab++; // count as gab point
	  continue;
	}
	int v = idx * inv_width_; // calculate y coordinate in image
	if ( v < 0 || v >= (int)cloud.height || pcl_isnan(cloud.points[idx].z)) // check left, right
	{
	  gab++; // count as gab point
	  continue;
	}
	float d = fabs(cloud.points[idx].z - p.z); 
	if ( d > distance_threshold )
	{
	  gab++; // count as gab point
	  //if ( d > 3.0*distance_threshold )
	  if ( d > distance_threshold )
	    *it_rbc += 1; // increment range border counter of current circle
	  continue;
	}
	if ( gab <= max_gab && has_prev_point) // check gab is small enough and a previous point exists
	{
	  p_curr = cloud.points[idx].getVector3fMap() - p.getVector3fMap();
	  ni = p_prev.cross(p_curr); // compute normal of p_prev and p_curr
	  ni = ni.normalized();
	  n_idx += ni; // add to mean normal
	  normals.push_back(ni);
	  p_prev = p_curr;
	}
	else // current is first point in circle or just after a gab
	{
	  p_prev = cloud.points[idx].getVector3fMap() - p.getVector3fMap();
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
	ni = p_prev.cross(p_first);
	ni = ni.normalized();
	n_idx += ni;
	normals.push_back(ni);
      }
    } // end loop of circles
  }

  if (range_border_counter[mask_.size()-1] > 0)
    label_out = I_BORDER;

  float num_p_inv = 1.0f / normals.size();
  n_idx *= num_p_inv;
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
    computePointNormal(*surface_, *it, 
		       output.points[*it].normal[0],
		       output.points[*it].normal[1],
		       output.points[*it].normal[2],
		       labels_->points[*it].label);
  }
}

#define PCL_INSTANTIATE_OrganizedNormalEstimation(T,OutT,LabelT) template class PCL_EXPORTS cob_3d_mapping_features::OrganizedNormalEstimation<T,OutT,LabelT>;

#endif
