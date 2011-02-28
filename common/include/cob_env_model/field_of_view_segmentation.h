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

#ifndef FIELDOFVIEWSEGMENTATION_H_
#define FIELDOFVIEWSEGMENTATION_H_

//##################
//#### includes ####

// standard includes
//--

// PCL includes
#include <pcl/pcl_base.h>

#include <Eigen/Core>

namespace ipa_env_model
{
	template <typename PointT>
	class FieldOfViewSegmentation: public pcl::PCLBase<PointT> {
		public:
			FieldOfViewSegmentation() {};
			//virtual ~FieldOfViewSegmentation();
			void computeFieldOfView(double fovHorizontal, double fovVertical, double maxRange,
					Eigen::Vector3d &n_up, Eigen::Vector3d &n_down, Eigen::Vector3d &n_right, Eigen::Vector3d &n_left);
			void segment(pcl::PointIndices &indices,
					Eigen::Vector3d &n_up, Eigen::Vector3d &n_down, Eigen::Vector3d &n_right, Eigen::Vector3d &n_left, double maxRange);
		protected:
			using pcl::PCLBase<PointT>::input_;

	};


	template <typename PointT>
	void FieldOfViewSegmentation<PointT>::computeFieldOfView(double fovHorizontal, double fovVertical, double maxRange,
			Eigen::Vector3d &n_up, Eigen::Vector3d &n_down, Eigen::Vector3d &n_right, Eigen::Vector3d &n_left)
	{
		Eigen::Vector3d vec_a;
		Eigen::Vector3d vec_b;
		Eigen::Vector3d vec_c;
		Eigen::Vector3d vec_d;

		/*Eigen::Vector3d n_up;
		Eigen::Vector3d n_down;
		Eigen::Vector3d n_right;
		Eigen::Vector3d n_left;
		Eigen::Vector3d n_front;*/

		double fovHorFrac = fovHorizontal/2;
		double fovVerFrac = fovVertical/2;

		vec_a(0) = tan(fovHorFrac)*maxRange;
		vec_a(1) = -tan(fovVerFrac)*maxRange;
		vec_a(2) = maxRange;

		vec_b(0) = -vec_a(0);
		vec_b(1) = vec_a(1);
		vec_b(2) = maxRange;

		vec_c(0) = -vec_a(0);
		vec_c(1) = -vec_a(1);
		vec_c(2) = maxRange;

		vec_d(0) = vec_a(0);
		vec_d(1) = -vec_a(1);
		vec_d(2) = maxRange;

		/*std::cout << "a: " << vec_a << std::endl;
		std::cout << "b: " << vec_b << std::endl;
		std::cout << "c: " << vec_c << std::endl;
		std::cout << "d: " << vec_d << std::endl;*/

		/// cross product b x a
		n_up(0)     = 0;//vec_a(1)*vec_b(2) - vec_a(2)*vec_b(1);
		n_up(1)     = vec_a(2)*vec_b(0) - vec_a(0)*vec_b(2);
		n_up(2)     = vec_a(0)*vec_b(1) - vec_a(1)*vec_b(0);

		/// cross product c x d
		/*n_down(0) = vec_c(1)*vec_d(2) - vec_c(2)*vec_d(1);
		n_down(1) = vec_c(2)*vec_d(0) - vec_c(0)*vec_d(2);
		n_down(2) = vec_c(0)*vec_d(1) - vec_c(1)*vec_d(0);*/
		n_down(0) = 0;
		n_down(1) = -n_up(1);
		n_down(2) = n_up(2);

		/// cross product b x c
		n_left(0)  = vec_b(1)*vec_c(2) - vec_b(2)*vec_c(1);
		n_left(1)  = 0;//vec_b(2)*vec_c(0) - vec_b(0)*vec_c(2);
		n_left(2)  = vec_b(0)*vec_c(1) - vec_b(1)*vec_c(0);

		/// cross product d x a
		/*n_right(0)   = vec_d(1)*vec_a(2) - vec_d(2)*vec_a(1);
		n_right(1)   = vec_d(2)*vec_a(0) - vec_d(0)*vec_a(2);
		n_right(2)   = vec_d(0)*vec_a(1) - vec_d(1)*vec_a(0);*/
		n_right(0) = -n_left(0);
		n_right(1) = 0;
		n_right(2) = n_left(2);

		/*std::cout << "n_up: " << n_up << std::endl;
		std::cout << "n_down: " << n_down << std::endl;
		std::cout << "n_right: " << n_right << std::endl;
		std::cout << "n_left: " << n_left << std::endl;*/

	}

	template <typename PointT>
	void FieldOfViewSegmentation<PointT>::segment(pcl::PointIndices &indices,
			Eigen::Vector3d &n_up, Eigen::Vector3d &n_down, Eigen::Vector3d &n_right, Eigen::Vector3d &n_left, double maxRange)
	{
		indices.header = input_->header;
		for(unsigned int i = 0; i<input_->points.size(); i++)
		{
			const PointT* curPoint = &input_->points[i];
			if(curPoint->y*n_up(1)+curPoint->z*n_up(2)<0 &&
					curPoint->y*n_down(1)+curPoint->z*n_down(2)<0 &&
					curPoint->x*n_right(0)+curPoint->z*n_right(2)<0 &&
					curPoint->x*n_left(0)+curPoint->z*n_left(2)<0 &&
					curPoint->z < maxRange)
				indices.indices.push_back(i);
		}
	}

} // end namespace ipa_env_model

#endif /* FIELDOFVIEWSEGMENTATION_H_ */
