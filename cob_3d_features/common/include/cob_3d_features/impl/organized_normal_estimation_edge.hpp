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
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de, Richard Bormann
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
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

#ifndef __IMPL_ORGANIZED_NORMAL_ESTIMATION_EDGE_H__
#define __IMPL_ORGANIZED_NORMAL_ESTIMATION_EDGE_H__

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_features/organized_normal_estimation_edge.h"
#include <pcl/common/eigen.h>


template<typename PointInT, typename PointOutT, typename LabelOutT> void cob_3d_features::OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>::computePointAngleLookupTable(int angular_bins)
{
	angular_bins_ = angular_bins;
	neighborhood_angles_ = cv::Mat(2*pixel_search_radius_+1,2*pixel_search_radius_+1,CV_32SC1);
	for (int dv=-pixel_search_radius_; dv<=pixel_search_radius_; ++dv)
	{
		for (int du=-pixel_search_radius_; du<=pixel_search_radius_; ++du)
		{
			double angle_rad = CV_PI*(1.-1./(double)angular_bins) + atan2((double)dv,(double)du);
			if (angle_rad < 0)
				angle_rad += 2*CV_PI;
			neighborhood_angles_.at<int>(dv+pixel_search_radius_, du+pixel_search_radius_) = std::max(0, std::min(angular_bins-1, (int)(angle_rad/(2.*CV_PI) * (double)angular_bins)));
		}
	}

//	for (int v=0; v<neighborhood_angles_.rows; ++v)
//	{
//		for (int u=0; u<neighborhood_angles_.cols; ++u)
//			std::cout << neighborhood_angles_.at<int>(v,u) << "\t";
//		std::cout << std::endl;
//	}
}

template<typename PointInT, typename PointOutT, typename LabelOutT> void cob_3d_features::OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>::computeSectorVisibility(int u, int v, float* visibility)
{
	for (int i=0; i<angular_bins_; ++i)
		visibility[i] = 4*pixel_search_radius_*pixel_search_radius_;

	int min_du = std::min(u, pixel_search_radius_);
	int max_du = std::min(edge_image_.cols-u-1, pixel_search_radius_);
	int min_dv = std::min(v, pixel_search_radius_);
	int max_dv = std::min(edge_image_.rows-v-1, pixel_search_radius_);
	uchar* edge_ptr;
	int* angle_ptr;
	// check all neighborhood pixels whether they are valid points for normal computation, i.e. no edge is on direct line of sight between nbh. pixel and central pixel
	for (int dv=-min_dv; dv<=max_dv; ++dv)
	{
		edge_ptr = edge_image_.ptr(v+dv) + u-min_du;
		angle_ptr = (int*)neighborhood_angles_.ptr(dv+pixel_search_radius_);
		for (int du=-min_du; du<=max_du; ++du, ++edge_ptr, ++angle_ptr)
		{
			if (du==0 && dv==0)
				continue;
			//if (edge_image_.at<uchar>(v+dv,u+du)!=0)
			if (*edge_ptr != 0)
			{
				// nbh. pixel itself is an edge pixel -> not usable and all pixels in the same sector with longer distance as well
				visibility[*angle_ptr] = std::min((float)du*du+dv*dv, visibility[*angle_ptr]);
			}
		}
	}
}

template<typename PointInT, typename PointOutT, typename LabelOutT> void cob_3d_features::OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>::computeNeighborhoodVisibility(int u, int v, bool* visibility)
{
	//const int pixel_search_radius_ = 8;
	const int step_width = 2*pixel_search_radius_+1;
	const int step_width2 = step_width*step_width;

	//bool visibility[step_width2];
	for (int i=0; i<step_width2; ++i)
		visibility[i] = true;

	int min_du = std::min(u, pixel_search_radius_);
	int max_du = std::min(edge_image_.cols-u-1, pixel_search_radius_);
	int min_dv = std::min(v, pixel_search_radius_);
	int max_dv = std::min(edge_image_.rows-v-1, pixel_search_radius_);
	uchar* edge_ptr;
	// check all neighborhood pixels whether they are valid points for normal computation, i.e. no edge is on direct line of sight between nbh. pixel and central pixel
	for (int dv=-min_dv; dv<=max_dv; ++dv)
	{
		edge_ptr = edge_image_.ptr(v+dv) + u-min_du;
		for (int du=-min_du; du<=max_du; ++du, ++edge_ptr)
		{
			if (du==0 && dv==0)
				continue;

			int index = (dv+pixel_search_radius_)*step_width+du+pixel_search_radius_;
			//if (edge_image_.at<uchar>(v+dv,u+du)!=0)
			if (*edge_ptr)
			{
				// nbh. pixel itself is an edge pixel -> not usable
				visibility[index] = false;
			}
			else
			{
				// check direct line of sight between nbh. pixel and central pixel for edges
				float curr_v = v+dv, curr_u = u+du;
				float su, sv;
				int adu = abs(du);
				int adv = abs(dv);
				int iterations;
				if (adu == adv)
				{
					su = isign(du);
					sv = isign(dv);
					iterations = adu;
				}
				else if (adu > adv)
				{
					su = isign(du);
					sv = -(float)dv/(float)adu;
					iterations = adu;
					curr_v += 0.5f*sign(dv);
				}
				else
				{
					su = -(float)du/(float)adv;
					sv = isign(dv);
					iterations = adv;
					curr_u += 0.5f*sign(du);
				}

				for (int i=0; i<iterations; ++i)
				{
					if (edge_image_.at<uchar>(curr_v,curr_u)!=0)
					{
						visibility[index] = false;
						break;
					}
					curr_u += su;
					curr_v += sv;
				}
			}
		}
	}
}

template<typename PointInT, typename PointOutT, typename LabelOutT> bool cob_3d_features::OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>::computePointVisibility(const int u, const int v, const int du, const int dv)
{
	if (edge_image_.at<uchar>(v+dv,u+du)!=0)
	{
		// nbh. pixel itself is an edge pixel -> not usable
		return false;
	}
	else
	{
		// check direct line of sight between nbh. pixel and central pixel for edges
		float curr_v = v+dv, curr_u = u+du;
		float su, sv;
		int adu = abs(du);
		int adv = abs(dv);
		int iterations;
		if (adu == adv)
		{
			su = isign(du);
			sv = isign(dv);
			iterations = adu;
		}
		else if (adu > adv)
		{
			su = isign(du);
			sv = -(float)dv/(float)adu;
			iterations = adu;
			curr_v += 0.5f*sign(dv);
		}
		else
		{
			su = -(float)du/(float)adv;
			sv = isign(dv);
			iterations = adv;
			curr_u += 0.5f*sign(du);
		}

		for (int i=0; i<iterations; ++i)
		{
			if (edge_image_.at<uchar>(curr_v,curr_u)!=0)
			{
				return false;
			}
			curr_u += su;
			curr_v += sv;
		}
	}

	return true;
}

#ifdef NEIGHBORHOOD_DISPLAY
template<typename PointInT, typename PointOutT, typename LabelOutT> void cob_3d_features::OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>::computePointNormal(
		const PointCloudIn &cloud, int index, float &n_x, float &n_y, float &n_z, int &label_out, cv::Mat& considered_neighborhood)
#else
template<typename PointInT, typename PointOutT, typename LabelOutT> void cob_3d_features::OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>::computePointNormal(
		const PointCloudIn &cloud, int index, float &n_x, float &n_y, float &n_z, int &label_out)
#endif
{
	//two vectors computed in the tangential plane: origin of vectors = query point,
	//end points are two points at the boundary of the neighborhood.
	//the normal is the cross product of those two vectors
	//
	//input: index - index of point in input_ cloud
	//output: n_x, n_y, n_z - coordinates of normal vector
	//---------------------------------------------------------------------------------------------------------

	const Eigen::Vector3f p = cloud.points[index].getVector3fMap();
	if (pcl_isnan(p(2)))
	{
		n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
		label_out = I_NAN;
		return;
	}
	const float distance_threshold = skip_distant_point_threshold_ * 0.003 * p(2) * p(2);

	const int idx_x = index % input_->width;
	const int idx_y = index * inv_width_;

	//no normal estimation if point is directly on edge
	if(!edge_image_.empty() && edge_image_.at<uchar>(idx_y,idx_x) != 0)
	{
		n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
		label_out = I_EDGE;
		return;
	}

	// verify validity of neighborhood points
//	const int step_width = 2*pixel_search_radius_+1;
//	const int step_width2 = step_width*step_width;
//	bool visibility[step_width2];
//	computeNeighborhoodVisibility(idx_x, idx_y, visibility);
	// verify validity of neighborhood points with angular sectors
	float visibility[angular_bins_];
	computeSectorVisibility(idx_x, idx_y, visibility);

	//std::vector<int> range_border_counter(mask_.size(), 0);
	Eigen::Vector3f p_curr;
	Eigen::Vector3f p_prev(0, 0, 0);
	Eigen::Vector3f p_first(0, 0, 0);
	Eigen::Vector3f n_idx(0, 0, 0);

	std::vector<std::vector<int> >::iterator it_c; // circle iterator
	std::vector<int>::iterator it_ci; // points in circle iterator
	//std::vector<int>::iterator it_rbc = range_border_counter.begin();

	//int n_normals = 0;

	// check where query point is and use out-of-image validation for neighbors or not
	if (idx_y >= pixel_search_radius_ && idx_y < (int)cloud.height - pixel_search_radius_ && idx_x >= pixel_search_radius_ && idx_x < (int)cloud.width - pixel_search_radius_)
	{
		//iterate over circles with decreasing radius (from pixel_search_radius to 0) -> cover entire circular neighborhood from outside border to inside
		//compute normal for every pair of points on every circle (that is a specific distance to query point)
		for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c) //, ++it_rbc) // iterate circles
		{
			bool has_prev_point = false;
			int init_gap = 0, gap = 0;
			//don't compute cross product, if the two tangential vectors are more than a quarter circle apart (prevent cross product of parallel vectors)
			const int max_gap = 0.25 * (*it_c).size(); // reset loop

			for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
			{
				const int idx = index + *it_ci;
				const Eigen::Vector3f p_i = cloud.points[idx].getVector3fMap();
				if (pcl_isnan(p_i(2)))
				{
					++gap;
					continue;
				} // count as gap point
				const int idx_du = (idx % input_->width - idx_x);
				const int idx_dv = (idx * inv_width_ - idx_y);
//				int vis_idx = (idx_dv+pixel_search_radius_)*step_width+idx_du+pixel_search_radius_;
				if (fabs(p_i(2) - p(2)) > distance_threshold ||		// todo: might be obsolete with edge image
						//visibility[vis_idx] == false)
						//computePointVisibility(idx_x, idx_y, idx_du, idx_dv) == false)
						visibility[neighborhood_angles_.at<int>(idx_dv+pixel_search_radius_, idx_du+pixel_search_radius_)] <= idx_du*idx_du+idx_dv*idx_dv)
				{
					++gap;
					//++(*it_rbc);
					continue;
				} // count as gap point
#ifdef NEIGHBORHOOD_DISPLAY
				considered_neighborhood.at<uchar>(idx_dv+pixel_search_radius_, idx_du+pixel_search_radius_) = 255;
#endif
				if (gap <= max_gap && has_prev_point) // check gap is small enough and a previous point exists
				{
					p_curr = p_i - p;
					n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
					//++n_normals;
					p_prev = p_curr;
				}
				else // current is first point in circle or just after a gap
				{
					p_prev = p_i - p;
					if (!has_prev_point)
					{
						p_first = p_prev; // remember the first valid point in circle
						init_gap = gap; // save initial gap size
						has_prev_point = true;
					}
				}
				gap = 0; // found valid point, reset gap counter
			}

			// close current circle (last and first point) if gap is small enough
			if (gap + init_gap <= max_gap)
			{
				// compute normal of p_first and p_prev
				n_idx += (p_prev.cross(p_first)).normalized();
				//++n_normals;
			}
		} // end loop of circles
	}
	else
	{
		for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c) //, ++it_rbc) // iterate circles
		{
			bool has_prev_point = false;
			int init_gap = 0, gap = 0;
			const int max_gap = 0.25 * (*it_c).size(); // reset circle loop

			for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
			{
				const int idx = index + *it_ci;
				// check top and bottom image border
				if (idx < 0 || idx >= (int)cloud.points.size())
				{
					++gap;
					continue;
				} // count as gap point
				const int v = idx * inv_width_; // calculate y coordinate in image, // check left, right border
				if (v < 0 || v >= (int)cloud.height || pcl_isnan(cloud.points[idx].z))
				{
					++gap;
					continue;
				} // count as gap point
				const int idx_du = (idx % input_->width - idx_x);
				const int idx_dv = (v - idx_y);
//				int vis_idx = (idx_dv+pixel_search_radius_)*step_width+idx_du+pixel_search_radius_;
				const Eigen::Vector3f p_i = cloud.points[idx].getVector3fMap();
				if (fabs(p_i(2) - p(2)) > distance_threshold ||		// todo: might be obsolete to test when edge is used
						//visibility[vis_idx] == false)
						//computePointVisibility(idx_x, idx_y, idx_du, idx_dv) == false)
						idx_du < pixel_search_radius_ || idx_du+pixel_search_radius_ >= neighborhood_angles_.cols ||
						visibility[neighborhood_angles_.at<int>(idx_dv+pixel_search_radius_, idx_du+pixel_search_radius_)] <= idx_du*idx_du+idx_dv*idx_dv)
				{
					++gap;
					//++(*it_rbc);
					continue;
				} // count as gab point
#ifdef NEIGHBORHOOD_DISPLAY
				considered_neighborhood.at<uchar>(idx_dv+pixel_search_radius_, idx_du+pixel_search_radius_) = 0;
#endif
				if (gap <= max_gap && has_prev_point) // check gab is small enough and a previous point exists
				{
					p_curr = p_i - p;
					n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
					//++n_normals;
					p_prev = p_curr;
				}
				else // current is first point in circle or just after a gap
				{
					p_prev = p_i - p;
					if (!has_prev_point)
					{
						p_first = p_prev; // remember the first valid point in circle
						init_gap = gap; // save initial gab size
						has_prev_point = true;
					}
				}
				gap = 0; // found valid point, reset gab counter
			}

			// close current circle (last and first point) if gab is small enough
			if (gap + init_gap <= max_gap)
			{
				// compute normal of p_first and p_prev
				n_idx += (p_prev.cross(p_first)).normalized();
				//++n_normals;
			}
		} // end loop of circles
	}

	//if (range_border_counter[mask_.size()-1] > 0) label_out = I_BORDER;

//	n_idx /= (float)n_normals;
//	n_idx = n_idx.normalized();
	n_idx.normalize();
	n_x = n_idx(0);
	n_y = n_idx(1);
	n_z = n_idx(2);
}


#define PCL_INSTANTIATE_OrganizedNormalEstimationEdge(T,OutT,LabelT) template class PCL_EXPORTS cob_3d_features::OrganizedNormalEstimationEdge<T,OutT,LabelT>;

#endif
