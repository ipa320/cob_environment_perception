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

#ifndef __ORGANIZED_NORMAL_ESTIMATION_EDGE_H__
#define __ORGANIZED_NORMAL_ESTIMATION_EDGE_H__

#include "cob_3d_features/organized_features.h"
#include "cob_3d_features/organized_normal_estimation.h"
#include <opencv2/opencv.hpp>

namespace cob_3d_features
{

int isign(int x)
{
	if (x==0)
		return 0;
	else if (x<0)
		return 1;
	return -1;
}

int sign(int x)
{
	if (x==0)
		return 0;
	else if (x<0)
		return -1;
	return 1;
}

//#define NEIGHBORHOOD_DISPLAY

template<typename PointInT, typename PointOutT, typename LabelOutT>
class OrganizedNormalEstimationEdge: public OrganizedNormalEstimation<PointInT, PointOutT, LabelOutT>
{
public:

	using OrganizedFeatures<PointInT, PointOutT>::pixel_search_radius_;
	using OrganizedFeatures<PointInT, PointOutT>::pixel_steps_;
	using OrganizedFeatures<PointInT, PointOutT>::circle_steps_;
	using OrganizedFeatures<PointInT, PointOutT>::inv_width_;
	using OrganizedFeatures<PointInT, PointOutT>::mask_;
	using OrganizedFeatures<PointInT, PointOutT>::input_;
	using OrganizedFeatures<PointInT, PointOutT>::indices_;
	using OrganizedFeatures<PointInT, PointOutT>::surface_;
	using OrganizedFeatures<PointInT, PointOutT>::skip_distant_point_threshold_;
	using OrganizedFeatures<PointInT, PointOutT>::feature_name_;
	using OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::labels_;

	typedef pcl::PointCloud<PointInT> PointCloudIn;
	typedef pcl::PointCloud<PointOutT> PointCloudOut;

	OrganizedNormalEstimationEdge()
	{
		feature_name_ = "OrganizedNormalEstimationEdge";
	}
	;

	inline void setEdgeImage(const cv::Mat& eIm)
	{
		edge_image_ = eIm;
	}

	void computePointAngleLookupTable(int angular_bins=16);

	// check all neighborhood pixels whether they are valid points for normal computation, i.e. no edge is on direct line of sight between nbh. pixel and central pixel
	void computeSectorVisibility(int u, int v, float* visibility);

	// labels all neighborhood pixels whether they are visible from the central pixel, i.e. no edges obstruct the direct line between neighbor point and central point
	void computeNeighborhoodVisibility(int u, int v, bool* visibility);

	bool computePointVisibility(const int u, const int v, const int du, const int dv);

#ifdef NEIGHBORHOOD_DISPLAY
	void computePointNormal(const PointCloudIn &cloud, int index, float &n_x, float &n_y, float &n_z, int &label_out, cv::Mat& considered_neighborhood);
#else
	void computePointNormal(const PointCloudIn &cloud, int index, float &n_x, float &n_y, float &n_z, int &label_out);
#endif

protected:

	cv::Mat edge_image_;
	cv::Mat neighborhood_angles_;  // stores the angle between a neighborhood point and its central point encoded as a bin number among angular_bins_ number of bins in 360deg
	int angular_bins_;		// 360deg are divided in this number of discrete angular bins
};
}

#endif
