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

#ifndef __IMPL_ORGANIZED_NORMAL_ESTIMATION_EDGE_OMP_H__
#define __IMPL_ORGANIZED_NORMAL_ESTIMATION_EDGE_OMP_H__

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_features/organized_normal_estimation_edge_omp.h"

#ifdef NEIGHBORHOOD_DISPLAY
#include <opencv2/highgui/highgui.hpp>
#endif

template<typename PointInT, typename PointOutT, typename LabelOutT> void cob_3d_features::OrganizedNormalEstimationEdgeOMP<PointInT, PointOutT, LabelOutT>::computeFeature(
		PointCloudOut &output)
{
	if (labels_->points.size() != input_->size())
	{
		labels_->points.resize(input_->size());
		labels_->height = input_->height;
		labels_->width = input_->width;
	}

#ifdef NEIGHBORHOOD_DISPLAY
	// neighborhood display
	cv::Mat neighborhood_display(input_->height, input_->width, CV_8UC3);
	int index = 0;
	for (int v=0; v<neighborhood_display.rows; ++v)
		for (int u=0; u<neighborhood_display.cols; ++u, ++index)
		{
			//if (u!=neighborhood_display.cols-1 && v!=neighborhood_display.rows-1)
			{
				pcl::PointXYZRGB* point = (pcl::PointXYZRGB*)&(input_->at(index));
				neighborhood_display.at<cv::Vec3b>(v,u) = cv::Vec3b(point->r, point->g, point->b);
			}
		}
	std::cout << "a" << std::endl;
#endif

#ifndef NEIGHBORHOOD_DISPLAY
	//const int threadsize = 1;
#pragma omp parallel for //schedule (dynamic, threadsize)
#endif
	for (size_t i = 0; i < indices_->size(); ++i)
	{
		labels_->points[(*indices_)[i]].label = I_UNDEF;
#ifndef NEIGHBORHOOD_DISPLAY
		this->computePointNormal(*surface_, (*indices_)[i], output.points[(*indices_)[i]].normal[0], output.points[(*indices_)[i]].normal[1], output.points[(*indices_)[i]].normal[2],
				labels_->points[(*indices_)[i]].label);
#else
		cv::Mat considered_neighborhood = cv::Mat::zeros(2*this->pixel_search_radius_+1, 2*this->pixel_search_radius_+1, CV_8UC1);
		this->computePointNormal(*surface_, (*indices_)[i], output.points[(*indices_)[i]].normal[0], output.points[(*indices_)[i]].normal[1], output.points[(*indices_)[i]].normal[2],
				labels_->points[(*indices_)[i]].label, considered_neighborhood);

		int u = i%neighborhood_display.cols;
		int v = i/neighborhood_display.cols;
		if (u%(3*this->pixel_search_radius_)==0 && v%(3*this->pixel_search_radius_)==0 && u>this->pixel_search_radius_ && u<neighborhood_display.cols-this->pixel_search_radius_ && v>this->pixel_search_radius_ && v<neighborhood_display.rows-this->pixel_search_radius_)
		{
			for (int dv=-this->pixel_search_radius_; dv<=this->pixel_search_radius_; ++dv)
				for (int du=-this->pixel_search_radius_; du<=this->pixel_search_radius_; ++du)
					if (considered_neighborhood.at<uchar>(dv+this->pixel_search_radius_, du+this->pixel_search_radius_) != 0)
						neighborhood_display.at<cv::Vec3b>(v+dv, u+du) = cv::Vec3b(neighborhood_display.at<cv::Vec3b>(v+dv, u+du).val[0], 255, neighborhood_display.at<cv::Vec3b>(v+dv, u+du).val[2]);
			neighborhood_display.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 255);
		}
#endif
	}

#ifdef NEIGHBORHOOD_DISPLAY
	std::cout << "b" << std::endl;
	cv::imshow("neighborhoods", neighborhood_display);
	cv::waitKey();
#endif
}

#define PCL_INSTANTIATE_OrganizedNormalEstimationEdgeOMP(T,OutT,LabelT) template class PCL_EXPORTS cob_3d_features::OrganizedNormalEstimationEdgeOMP<T,OutT,LabelT>;

#endif

