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
#include "gtest/gtest.h"


template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
pcl::PolygonMesh::Ptr cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_subsampling_of_Map(const int num, const Scalar r2) {
	//generate random map
	triangulated_input_.clear();
	for(int i=0; i<num; i++) {
		//some model
		typename TSurface::Model model;
		model.p = model.p.Random();

		//generate random triangle
		Triangle t;
		for(int j=0; j<3; j++) t.p_[j] = t.p_[j].Random();
		t.model_ = &model;
		t.compute(radii_);
		triangulated_input_.push_back(t);
	}
	
	//select some point and generate mesh
	std::vector<Triangle> res;
    subsample(triangulated_input_[rand()%triangulated_input_.size()].p3_[rand()%3], r2, res);
	
	return dbg_triangles2mesh(res);
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
bool cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_singleTriangle(const int num) const {

	bool zero_model = false, random_triangles=true;
	int test = 1;

	for(int i=0; i<num; i++) {
		//some model
		typename TSurface::Model model;
		if(zero_model)
			model.p.fill(0);
		else
			model.p = model.p.Random();

		//generate random triangle
		Triangle t;
		if(random_triangles) {
			for(int j=0; j<3; j++) t.p_[j] = t.p_[j].Random();
		} else {
			t.p_[0](0)=0;
			t.p_[0](1)=0;
			t.p_[1](0)=0;
			t.p_[1](1)=1;
			t.p_[2](0)=1;
			t.p_[2](1)=1;
		}
		t.model_ = &model;
		t.compute(radii_);

		//subdivide triangle to 3 and re run
		//sum should be equal to first triangle
		Triangle t1 = t, t2=t, t3=t;
		t1.p_[0] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
		t2.p_[1] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
		t3.p_[2] = (t.p_[0]+t.p_[1]+t.p_[2])/3;

		t1.compute(radii_);
		t2.compute(radii_);
		t3.compute(radii_);

		if(test==0) {
			Scalar m=1,n=0,p=0;

			for(m=0; m<=1; m++)
			for(n=0; n<=1; n++)
			for(p=0; p<=1; p++)
			{
				std::complex<Scalar> c, c2;

				std::cout<<"mnp "<<m<<" "<<n<<" "<<p <<"    ------------------------"<<std::endl;
				c = t1.kernel(m,n,p);
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				c = t2.kernel(m,n,p);
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				c = t3.kernel(m,n,p);
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				c = (t1.kernel(m,n,p)+t2.kernel(m,n,p)+t3.kernel(m,n,p));
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				c2 = t.kernel(m,n,p);
				std::cout<< c2<<" "<<std::abs(c) <<std::endl<<std::endl;
				std::cout<<"----------------------------------------"<<std::endl;

				EXPECT_NEAR(c.real(),c2.real(), 0.001);
				EXPECT_NEAR(c.imag(),c2.imag(), 0.001);
				EXPECT_TRUE(c==c);	//nan test
			}
		}
		else if(test==1) {
			FeatureAngleComplex sum1, sum2; sum1.fill(0); sum2.fill(0);
			for(size_t j=0; j<radii_.size(); j++)
				for(size_t k=0; k<num_radius_; k++) {
					sum1 += t1.f_[j].vals[k]+t2.f_[j].vals[k]+t3.f_[j].vals[k];
					sum2 += t.f_[j].vals[k];

					std::cout<<std::endl<<(t1.f_[j].vals[k]+t2.f_[j].vals[k]+t3.f_[j].vals[k])<<std::endl;
					std::cout<<t.f_[j].vals[k]<<std::endl;
				}

			std::cout<<"----------------------------------------"<<std::endl;
			std::cout<<std::endl<<sum1<<std::endl<<std::endl;
			std::cout<<sum2<<std::endl;
			std::cout<<"----------------------------------------"<<std::endl;
			std::cout<<sum2-sum1<<std::endl;

			EXPECT_TRUE( sum1.isApprox(sum2, 0.001) );
			//EXPECT_TRUE( sum1.allFinite() );	//nan test

			if(sum1!=sum2) return false;
		}
		else return false;
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
