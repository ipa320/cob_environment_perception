/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by:
 *
 * Date of creation: 02/2011
 * ToDo:
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

#ifndef IMPL_FIELDOFVIEWSEGMENTATION_H_
#define IMPL_FIELDOFVIEWSEGMENTATION_H_

#include <cob_env_model/field_of_view_segmentation.h>



template <typename PointT>
void ipa_env_model::FieldOfViewSegmentation<PointT>::segment(pcl::PointIndices &indices,
		Eigen::Vector3d &n_up, Eigen::Vector3d &n_down, Eigen::Vector3d &n_right, Eigen::Vector3d &n_left, Eigen::Vector3d &n_origin, double maxRange)
{
	indices.header = input_->header;
	for(unsigned int i = 0; i<input_->points.size(); i++)
	{
		const PointT* curPoint = &input_->points[i];
		//TODO: check for max range
		if(n_up(0)*(curPoint->x-n_origin(0))+n_up(1)*(curPoint->y-n_origin(1))+n_up(2)*(curPoint->z-n_origin(2)) < 0 &&
				n_down(0)*(curPoint->x-n_origin(0))+n_down(1)*(curPoint->y-n_origin(1))+n_down(2)*(curPoint->z-n_origin(2)) < 0 &&
				n_right(0)*(curPoint->x-n_origin(0))+n_right(1)*(curPoint->y-n_origin(1))+n_right(2)*(curPoint->z-n_origin(2)) < 0 &&
				n_left(0)*(curPoint->x-n_origin(0))+n_left(1)*(curPoint->y-n_origin(1))+n_left(2)*(curPoint->z-n_origin(2)) < 0)
		{
			indices.indices.push_back(i);
			//std::cout << "Point "  << *curPoint << " is in FOV" << std::endl;
		}
	}
}


#define PCL_INSTANTIATE_FieldOfViewSegmentation(T) template class ipa_env_model::FieldOfViewSegmentation<T>;

#endif /* IMPL_FIELDOFVIEWSEGMENTATION_H_ */
