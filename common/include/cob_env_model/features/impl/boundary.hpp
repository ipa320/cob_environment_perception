/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: boundary.hpp 1667 2011-07-10 22:44:00Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_BOUNDARY_H_
#define PCL_FEATURES_IMPL_BOUNDARY_H_

#include "cob_env_model/features/boundary.h"
#include <cfloat>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
  ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>::isBoundaryPoint (
      const pcl::PointCloud<PointInT> &cloud, int q_idx, 
      const std::vector<int> &indices, 
      const Eigen::Vector3f &u, const Eigen::Vector3f &v, 
      float angle_threshold)
{
  return (isBoundaryPoint (cloud, cloud.points[q_idx], indices, u, v, angle_threshold));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>::isBoundaryPoint (
      const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point, 
      const std::vector<int> &indices, 
      const Eigen::Vector3f &u, const Eigen::Vector3f &v, 
      float angle_threshold)
{
  if (indices.size () < 3)
    return (false);
  float uvn_nn[2];
  Eigen::Vector3f delta;
  delta.setZero ();
  // Compute the angles between each neighboring point and the query point itself
  std::vector<float> angles;
  angles.reserve (indices.size ());
  for (size_t i = 0; i < indices.size (); ++i)
  {
    delta[0] = cloud.points[indices[i]].x - q_point.x;
    delta[1] = cloud.points[indices[i]].y - q_point.y;
    delta[2] = cloud.points[indices[i]].z - q_point.z;

    uvn_nn[0] = u.dot (delta);
    uvn_nn[1] = v.dot (delta);

    if (uvn_nn[0] == 0 && uvn_nn[1] == 0)
      continue;
    angles.push_back (atan2 (uvn_nn[1], uvn_nn[0])); // the angles are fine between -PI and PI too
  }
  std::sort (angles.begin (), angles.end ());

  // Compute the maximal angle difference between two consecutive angles
  float max_dif = FLT_MIN, dif;
  for (size_t i = 0; i < angles.size () - 1; ++i)
  {
    dif = angles[i + 1] - angles[i];
    if (max_dif < dif)
      max_dif = dif;
  }
  // Get the angle difference between the last and the first
  dif = 2 * M_PI - angles[angles.size () - 1] + angles[0];
  if (max_dif < dif)
    max_dif = dif;

  // Check results
  if (max_dif > angle_threshold)
    return (true);
  else
    return (false);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>::isEdgePoint (
      const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point,
      const std::vector<int> &indices,
      const Eigen::Vector3f &n, const float &threshold)
{
	//TODO: choose threshold according to viewpoint distance (noise)
  if (indices.size () < 3)
    return (false);
  float nd_dot;
  //float c_ang_thresh = cos(ang_threshold);
  Eigen::Vector3f n_norm = n.normalized();
  Eigen::Vector3f delta;
  delta.setZero ();
  // Compute the angles between each neighboring point and the query point itself
  //std::vector<float> angles;
  //angles.reserve (indices.size ());
  int nn_ctr=0, nan_ctr=0;
  int b_ctr=0;
  //std::cout << "d: ";
  for (size_t i = 0; i < indices.size (); ++i)
  {
    delta[0] = cloud.points[indices[i]].x - q_point.x;
    delta[1] = cloud.points[indices[i]].y - q_point.y;
    delta[2] = cloud.points[indices[i]].z - q_point.z;
    //delta = delta.normalized();

    nd_dot = n_norm.dot (delta);

    //double nd_dot_sum;

    if(nd_dot==nd_dot)
    {
    	nn_ctr++;
    	if(fabs(nd_dot) > threshold) //is border point
    		b_ctr++;
    	//std::cout << nd_dot << ",";
    	//angles.push_back (fabs(nd_dot)); // the angles are fine between -PI and PI too
    	//nd_dot_sum += fabs(nd_dot);
    }
    else
    {
    	nan_ctr++;
    }
  }
  float edge_prob = (float)b_ctr/nn_ctr;
  float boundary_prob = (float)nan_ctr/indices.size();
  /*std::cout << border_prob << std::endl;
  if(border_prob >0.00001)
	  return true;
  else
	  return false;*/
  //double nd_dot_av = nd_dot_sum/nn_ctr;
  //std::sort (angles.begin (), angles.end ());

  //float max_angle = angles.back();
  //std::cout << max_angle << "," << acos(max_angle) << std::endl;

  //TODO: mark NaNs
  if(/*max_angle>threshold*/edge_prob>0.1 || boundary_prob>0.2)
  {
	  //std::cout << "true";
	  return (true);
  }
  else
  {
	  //std::cout << "false";
	  return (false);
  }

}

/*template <typename PointInT, typename PointNT, typename PointOutT> int
	ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>::searchForNeighbors (
			int index,
			double radius,
			std::vector<int>& indices,
			std::vector<float>& distances)
{
	indices.clear();
	distances.clear();
	const PointInT& p = input_->points[index];


	double dx = 0;
	int idx = index+1;
	//search in horizontal direction right
	while(dx < radius && idx%input_->width!=0)
	{
		dx = fabs(p.x-input_->points[idx].x);
		indices.push_back(idx);
		idx++;
		//break at end of line
		//if(idx%input_->width==0) break;
	}
	dx = 0;
	idx = index-1;
	//search in horizontal direction left
	while(dx < radius && (idx+1)%input_->width!=0)
	{
		dx = fabs(p.x-input_->points[idx].x);
		indices.push_back(idx);
		idx--;
		//break at beginning of line
		//if((idx-1)%input_->width==0) break;
	}
	//search in vertical direction up
	double dy = 0;
	idx = index-input_->width;
	while(idx >= 0 && dy < radius)
	{
		dy = fabs(p.y-input_->points[idx].y);
		indices.push_back(idx);
		idx-=input_->width;
		//break at top of image
		//if(idx<0) break;
	}
	//search in vertical direction down
	dy = 0;
	idx = index+input_->width;
	while(idx < input_->size() && dy < radius)
	{
		dy = fabs(p.y-input_->points[idx].y);
		indices.push_back(idx);
		idx+=input_->width;
		//break at top of image
		//if(idx >= input_->size()) break;
	}
	return 0;
}*/

template <typename PointInT, typename PointNT, typename PointOutT> int
	ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>::searchForNeighbors (
			int index,
			double radius,
			std::vector<int>& indices,
			std::vector<float>& distances)
{
	indices.clear();
	distances.clear();
	const PointInT& p = input_->points[index];
	int idx_x = index%input_->width;
	int idx_y = index/input_->width;
	int v=2;

	//TODO: choose search radius according to viewpoint distance
	//TODO: NaN tests
	//TODO: handle border points
	if(idx_x<v || idx_x>=input_->width-v || idx_y<v || idx_y>=input_->height-v)
		return 0;

	//get approximate x-y-resolution for p
	double r_x = fabs((input_->points[idx_y*input_->width+idx_x+v].x-input_->points[idx_y*input_->width+idx_x-v].x)/(2*v+1));
	double r_y = fabs((input_->points[(idx_y+v)*input_->width+idx_x].y-input_->points[(idx_y-v)*input_->width+idx_x].y)/(2*v+1));
	//double res = (r_x+r_y)/2;

	int num_nn = 4;
	int step_x = radius/r_x;
	int step_y = radius/r_y;
	//TODO: limit number of neighbors
	int incr_x = step_x/(num_nn-1);
	int incr_y = step_y/(num_nn-1);
	//std::cout << idx_x << "," << idx_y <<  " step: " << step << std::endl;
	//std::cout << idx_x << "," << idx_y << ": ";
	//TODO: Adjust stepping according to resolution (if resolution low, increase stepping)
	for (int i=idx_x-step_x; i<=idx_x+step_x; i++)
	{
		if(i>=input_->width) break;
		for (int j=idx_y-step_y; j<=idx_y+step_y; j++)
		{
			if(i==idx_x && j==idx_y) continue; //skip p itself
			if(j>=input_->height) break;
			indices.push_back(i+j*input_->width);
			//std::cout << "(" << i << "," << j << ")";
		}
	}
	//std::cout << std::endl;

	/*double dx = 0;
	int idx = index+1;
	//search in horizontal direction right
	while(dx < radius && idx%input_->width!=0)
	{
		dx = fabs(p.x-input_->points[idx].x);
		indices.push_back(idx);
		idx++;
		//break at end of line
		//if(idx%input_->width==0) break;
	}
	dx = 0;
	idx = index-1;
	//search in horizontal direction left
	while(dx < radius && (idx+1)%input_->width!=0)
	{
		dx = fabs(p.x-input_->points[idx].x);
		indices.push_back(idx);
		idx--;
		//break at beginning of line
		//if((idx-1)%input_->width==0) break;
	}
	//search in vertical direction up
	double dy = 0;
	idx = index-input_->width;
	while(idx >= 0 && dy < radius)
	{
		dy = fabs(p.y-input_->points[idx].y);
		indices.push_back(idx);
		idx-=input_->width;
		//break at top of image
		//if(idx<0) break;
	}
	//search in vertical direction down
	dy = 0;
	idx = index+input_->width;
	while(idx < input_->size() && dy < radius)
	{
		dy = fabs(p.y-input_->points[idx].y);
		indices.push_back(idx);
		idx+=input_->width;
		//break at top of image
		//if(idx >= input_->size()) break;
	}*/
	return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
	ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  Eigen::Vector3f u, v;

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
	  //TODO: test nn search
    if(this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists))
    {

		/*std::cout << idx << ": ";
		for (int i=0; i<nn_indices.size(); i++)
		{
			std::cout << nn_indices[i] << ",";
		}*/


		// Obtain a coordinate system on the least-squares plane
		getCoordinateSystemOnPlane (normals_->points[(*indices_)[idx]], u, v);

		Eigen::Vector3f normal;
		normal[0] = normals_->points[(*indices_)[idx]].normal_x;
		normal[1] = normals_->points[(*indices_)[idx]].normal_y;
		normal[2] = normals_->points[(*indices_)[idx]].normal_z;
		// Estimate whether the point is lying on a boundary surface or not
		//std::cout << idx << "," << idx%input_->width << ":";
		//TODO: test edge detection
		output.points[idx].boundary_point = isEdgePoint (*surface_, input_->points[(*indices_)[idx]], nn_indices, normal, angle_threshold_);
    }
    //std::cout << std::endl;
  }
}

#define PCL_INSTANTIATE_BoundaryEstimation(PointInT,PointNT,PointOutT) template class PCL_EXPORTS ipa_features::BoundaryEstimation<PointInT, PointNT, PointOutT>;

#endif    // PCL_FEATURES_IMPL_BOUNDARY_H_ 

