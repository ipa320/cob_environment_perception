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
 * ROS package name: cob_3d_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Joshua Hampp, email:joshua.hampp@ipa.fraunhofer.de
 *
 * Date of creation: 03/2014
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

#pragma once

#include "invariant_surface_feature.hpp"

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
bool cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_singleTriangle(const int num) const {

	for(int i=0; i<num; i++) {
		//some model
		typename TSurface::Model model;
		model.p = model.p.Random();

		//generate random triangle
		Triangle t;
		for(int j=0; j<3; j++) t.p_[j] = t.p_[j].Random();
t.p_[0](0)=0;
t.p_[0](1)=0;
t.p_[1](0)=0;
t.p_[1](1)=1;
t.p_[2](0)=1;
t.p_[2](1)=1;
		t.model_ = &model;
		t.compute(radii_);

		//t.print();

		//subdivide triangle to 3 and re run
		//sum should be equal to first triangle
		Triangle t1 = t, t2=t, t3=t;
		t1.p_[0] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
		t2.p_[1] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
		t3.p_[2] = (t.p_[0]+t.p_[1]+t.p_[2])/3;

		t1.compute(radii_);
		t2.compute(radii_);
		t3.compute(radii_);

		std::cout<< t.kernel(0,0,0) <<std::endl;
		std::cout<< t1.kernel(0,0,0) <<std::endl;
		std::cout<< t2.kernel(0,0,0) <<std::endl;
		std::cout<< t3.kernel(0,0,0) <<std::endl;
		std::cout<< (t1.kernel(0,0,0)+t2.kernel(0,0,0)+t3.kernel(0,0,0)) <<std::endl;

t1.print();
		/*t1.print();
		t2.print();
		t3.print();

		FeatureAngleComplex sum1, sum2; sum1.fill(0); sum2.fill(0);
		for(size_t j=0; j<radii_.size(); j++)
			for(size_t k=0; k<num_radius_; k++) {
				sum1 += t1.f_[j].vals[k]+t2.f_[j].vals[k]+t3.f_[j].vals[k];
				sum2 += t.f_[j].vals[k];
		std::cout<<std::endl<<(t1.f_[j].vals[k]+t2.f_[j].vals[k]+t3.f_[j].vals[k])<<std::endl;
		std::cout<<t.f_[j].vals[k]<<std::endl;
			}
		std::cout<<std::endl<<sum1<<std::endl<<std::endl;
		std::cout<<sum2<<std::endl;

		if(sum1!=sum2) return false;*/
	}

	return true;
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::print() const {
	std::cout<< "Triangle" <<std::endl;
	for(int i=0; i<3; i++)
		std::cout<<p_[i].transpose()<<std::endl;
/*	for(size_t i=0; i<f_.size(); i++) {
		for(size_t j=0; j<num_radius_; j++)
			std::cout<<f_[i].vals[j]<<std::endl;
	}*/
}
