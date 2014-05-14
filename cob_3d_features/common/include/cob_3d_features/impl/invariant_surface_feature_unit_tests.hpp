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
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_addOffset(const Scalar off_x, const Scalar off_y, const Scalar off_z) {
	for(size_t i=0; i<input_->size(); i++) {
		(*input_)[i].model_.p(0) += off_z;
		for(size_t j=0; j<(*input_)[i].segments_.size(); j++)
			for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++) {
				(*input_)[i].segments_[j][k](0) += off_x;
				(*input_)[i].segments_[j][k](1) += off_y;
			}
	}

	for(size_t i=0; i<triangulated_input_.size(); i++) {
		for(int j=0; j<3; j++) {
			triangulated_input_[i].p_[j](0) += off_x;
			triangulated_input_[i].p_[j](1) += off_y;
		}
		triangulated_input_[i].compute(radii_);
	}
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_rotate(const Scalar angle) {
	for(size_t i=0; i<triangulated_input_.size(); i++) {
		std::cout<<"AAAAA "<<i<<std::endl;
		triangulated_input_[i].print();
	}

	const Scalar s = std::sin(angle);
	const Scalar c = std::cos(angle);

	Eigen::Matrix<Scalar, 2,2> R3;
	Eigen::Matrix<float, 4,4> R, Ri;
	Eigen::Matrix3f R2;
	R.fill(0);
	R(0,0)=1;
	R(1,1)=c; R(1,2)=-s;
	R(2,1)=s; R(2,2)=c;
	Ri = R;
	Ri(2,1)*=-1;
	Ri(1,2)*=-1;

	R2.fill(0);
	R2(2,2)=1;
	R2(0,0)=c; R2(0,1)=-s;
	R2(1,0)=s; R2(1,1)=c;

	R3.fill(0);
	R3(0,0)=c; R3(0,1)=-s;
	R3(1,0)=s; R3(1,1)=c;

	for(size_t i=0; i<input_->size(); i++) {
		(*input_)[i].model_.p = Ri*(*input_)[i].model_.p;
		for(size_t j=0; j<(*input_)[i].segments_.size(); j++)
			for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++)
				(*input_)[i].segments_[j][k] = R2*(*input_)[i].segments_[j][k];
	}

	for(size_t i=0; i<triangulated_input_.size(); i++) {
		for(int j=0; j<3; j++)
			triangulated_input_[i].p_[j] = R3*triangulated_input_[i].p_[j];
		triangulated_input_[i].compute(radii_);
		std::cout<<"BBBBB "<<i<<std::endl;
		triangulated_input_[i].print();
	}
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
pcl::PolygonMesh::Ptr cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_subsampling_of_Map(const int num, const Scalar r2) {
	//generate random map
	input_.reset(new TSurfaceList);
	input_->resize(num);
	triangulated_input_.clear();
	for(int i=0; i<num; i++) {
		//some model
		typename TSurface::Model *model = &(*input_)[i].model_;
		model->p = model->p.Random();

		//generate random triangle
		Triangle t;
		(*input_)[i].segments_.resize(1);
		for(int j=0; j<3; j++) {
			t.p_[j] = t.p_[j].Random();
			Eigen::Vector3f v = Eigen::Vector3f::Zero();
			v.head<2>() = t.p_[j].template cast<float>();
			(*input_)[i].segments_[0].push_back(v);
		}
		t.model_ = model;
		t.compute(radii_);
		triangulated_input_.push_back(t);
	}
	
	//select some point and generate mesh
	std::vector<Triangle> res;
//    subsample(triangulated_input_[rand()%triangulated_input_.size()].p3_[rand()%3], r2, res);
    subsample(Eigen::Matrix<Scalar, 3, 1>::UnitX(), r2, res);

	return dbg_triangles2mesh(res);
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
bool cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::test_singleTriangle(const int num) const {

	bool zero_model = false, random_triangles=true;
	int test = 0, tests_succeeded=0, tests_num=0;

	for(int i=0; i<num; i++) {
		//some model
		typename TSurface::Model model;
		if(zero_model)
			model.p.fill(0);
		else
			model.p = model.p.Random();
		model.p(0) += 2;
//model.p(3)=model.p(4)=model.p(5)=0;

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
			Scalar m=0.9,n=0.4,p=0;

			for(m=0; m<=3; m++)
			for(n=0; n<=3; n++)
			for(p=0; p<=3; p++)
			{
				std::complex<Scalar> c, c2, c3;

				std::cout<<"mnp "<<m<<" "<<n<<" "<<p <<"    ------------------------"<<std::endl;
				c = t1.kernel(m,n,p);
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				c = t2.kernel(m,n,p);
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				c = t3.kernel(m,n,p);
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;

				std::cout<<"mnp "<<m<<" "<<n<<" "<<p <<"    -------------------11111"<<std::endl;
				c = (t1.kernel(m,n,p)+t2.kernel(m,n,p)+t3.kernel(m,n,p));
				std::cout<< c<<" "<<std::abs(c) <<std::endl<<std::endl;
				std::cout<<"mnp "<<m<<" "<<n<<" "<<p <<"    -------------------22222"<<std::endl;
				c2 = t.kernel(m,n,p);
				std::cout<< c2<<" "<<std::abs(c2) <<std::endl<<std::endl;
				std::cout<<"----------------------------------------"<<std::endl;

				t1.print();
				t2.print();
				t3.print();
				t.print();

				std::cout<<"model: "<<model.p.transpose()<<std::endl;

				/*if(std::abs(c-c2) > 0.01 ) {
					for(int j=0; j<3; j++) t.p_[j] += t.p_[j].Random()*0.01;
					t1 = t; t2=t; t3=t;
					t1.p_[0] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
					t2.p_[1] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
					t3.p_[2] = (t.p_[0]+t.p_[1]+t.p_[2])/3;
					std::cout<<"    -------------------tttttt"<<std::endl;
					c3 = (t1.kernel(m,n,p)+t2.kernel(m,n,p)+t3.kernel(m,n,p));
					std::cout<< c3<<" "<<std::abs(c3) <<std::endl<<std::endl;
				}*/

				EXPECT_NEAR(c.real(),c2.real(), 0.05);
				EXPECT_NEAR(c.imag(),c2.imag(), 0.05);
				EXPECT_TRUE(c==c);	//nan test

				tests_succeeded += (std::abs(c-c2) < 0.1&&c==c)?1:0;
				tests_num++;
				//assert(std::abs(c-c2) < 0.1);
				//assert(c==c);	//nan test

			}
		}
		else if(test==1) {
			FeatureAngleComplex sum1, sum2; sum1.fill(0); sum2.fill(0);
			for(size_t j=0; j<radii_.size(); j++)
				for(size_t k=0; k<num_radius_; k++) {
					sum1 += t1.f_[j].vals[k]+t2.f_[j].vals[k]+t3.f_[j].vals[k];
					sum2 += t.f_[j].vals[k];

					std::cout<<"==================="<<std::endl;
					std::cout<<std::endl<<(t1.f_[j].vals[k])<<std::endl<<std::endl;
					std::cout<<std::endl<<(t2.f_[j].vals[k])<<std::endl<<std::endl;
					std::cout<<std::endl<<(t3.f_[j].vals[k])<<std::endl<<std::endl;
					std::cout<<"###################"<<std::endl;
					std::cout<<std::endl<<(t1.f_[j].vals[k]+t2.f_[j].vals[k]+t3.f_[j].vals[k])<<std::endl<<std::endl;
					std::cout<<t.f_[j].vals[k]<<std::endl;
				}

			std::cout<<"----------------------------------------"<<std::endl;
			std::cout<<std::endl<<sum1<<std::endl<<std::endl;
			std::cout<<sum2<<std::endl;
			std::cout<<"----------------------------------------"<<std::endl;
			std::cout<<sum2-sum1<<std::endl;

			std::cout<<"model: "<<model.p.transpose()<<std::endl;

			EXPECT_TRUE( sum1.isApprox(sum2, 0.001) );
			//EXPECT_TRUE( sum1.allFinite() );	//nan test

			//if(sum1!=sum2) return false;
		}
		else return false;
	}

	std::cout<<"Tests: "<<tests_succeeded<<"/"<<tests_num<<std::endl;

	return tests_succeeded==tests_num;
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::print() const {
	std::cout<< "Triangle "<< model_ <<std::endl;
	for(int i=0; i<3; i++)
		std::cout<<p_[i].transpose()<<std::endl;
	for(int i=0; i<3; i++) {
		std::cout<<p3_[i].transpose()<<std::endl;
		assert(at(p_[i])==p3_[i]);
	}
/*	for(size_t i=0; i<f_.size(); i++) {
		for(size_t j=0; j<num_radius_; j++)
			std::cout<<f_[i].vals[j]<<std::endl;
	}*/
}
