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

#include "cob_3d_features/invariant_surface_feature.h"
#include <unsupported/Eigen/FFT>
#include <unsupported/Eigen/Polynomials>

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::compute() {
  result_.reset(new Result);

  //generate keypoints (e.g. reduce number of points by area)
  std::vector<TVector> keypoints;
  generateKeypoints(keypoints);
  std::cout<<"got "<<keypoints.size()<<" keypoints"<<std::endl;

  result_->resize(keypoints.size());
  for(size_t j=0; j<radii_.size(); j++) {
	for(size_t i=0; i<keypoints.size(); i++) {		
      //generate sub map
      std::vector<Triangle> submap;
      subsample(keypoints[i], radii_[j], submap);

	  //calc. rotation invariant feature
	  sr_.clear();
	  //sum features
	  for(size_t s=0; s<submap.size(); s++) {
		  std::cout<<s<<std::endl;
		  sr_ += submap[s];
	  }
		  
	  sr_.finish((*result_)[j]);
    }
  }
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::compute(const typename S::Samples &samples) {
	for(int i=0; i<3; i++) {
		this->p3_[i](0) = p_[i](0);
		this->p3_[i](1) = p_[i](1);
		this->p3_[i](2) = model_->model(p_[i](0), p_[i](1));
	}
	
	//check if further sub-sampling is necessary?
	if(area<TSurface::DEGREE>()>0.0001/*||rand()%13==0||depth<4*/) {
std::cout<<"subdivide "<<area<TSurface::DEGREE>()<<std::endl;
		Eigen::Matrix<Scalar, 2, 1> ps[3];
		for(int i=0; i<3; i++)
			ps[i] = ((p_[i])+(p_[(i+1)%3]))/2;

		Triangle tris[4];
		for(int i=0; i<3; i++) {
			tris[i].p_[i] = p_[i];
			tris[i].p_[(i+1)%3] = ps[i];
			tris[i].p_[(i+2)%3] = ps[(i+2)%3];
			tris[3].p_[i] = ps[i];
		}
		for(int i=0; i<4; i++) {
			tris[i].model_ = model_;
			tris[i].compute(samples);
			for(size_t j=0; j<this->f_.size(); j++)
				for(size_t k=0; k<this->f_[j].size(); k++)
					this->f_[j][k] += ((typename S::Values)tris[i])[j][k];
		}
	} else 
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, typename S::Samples, typename S::Values>::compute(samples);
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::generateKeypoints(std::vector<TVector> &keypoints) const {
  //TODO: reduce points

  /*for(size_t i=0; i<input_->size(); i++) {                         //surfaces
    for(size_t j=0; j<(*input_)[i].segments_.size(); j++) {        //outline/holes
      for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++) {   //points
        TVector v;
        v(0) = (*input_)[i].segments_[j][k](0);
        v(1) = (*input_)[i].segments_[j][k](1);
        v(2) = (*input_)[i].model_.model( v(0), v(1) );
        keypoints.push_back(v);
      }
    }
  }*/


  for(size_t i=0; i<input_->size(); i++) {                         //surfaces
    if((*input_)[i].segments_.size()<1) continue;        //outline/holes
      for(size_t k=0; k<(*input_)[i].segments_[0].size(); k+=20) {   //points
        TVector v;
        v(0) = (*input_)[i].segments_[0][k](0);
        v(1) = (*input_)[i].segments_[0][k](1);
        v(2) = (*input_)[i].model_.model( v(0), v(1) );
        keypoints.push_back(v);
        break;
      }
  }
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
Eigen::Matrix<Scalar, 2, 1>
cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::intersection_on_line(const TVector &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b) const {

assert((this->at(a)-at).squaredNorm()>r2 || (this->at(b)-at).squaredNorm()>r2);

	Eigen::PolynomialSolver<typename TSurface::Model::VectorU1D::Scalar, 2*TSurface::DEGREE> solver;	
	typename TSurface::Model::VectorU1D p = model_->transformation_1D( (b-a).template cast<typename TSurface::Model::VectorU1D::Scalar>(), a.template cast<typename TSurface::Model::VectorU1D::Scalar>(), at.template cast<typename TSurface::Model::VectorU1D::Scalar>());
	p(0) -= r2;
	solver.compute(p);

//std::cout<<"p "<<p.transpose()<<std::endl;
	
	std::vector<typename TSurface::Model::VectorU1D::Scalar> r;
	solver.realRoots(r);
	assert(r.size()>0);
	
	const typename TSurface::Model::VectorU1D::Scalar mid = 0.5f;
	size_t best=0;
	for(size_t i=1;i<r.size();++i)
	{
		if( (r[best]-mid)*(r[best]-mid)>(r[i]-mid)*(r[i]-mid))
			best=i;
	}

	assert(r[best]>=0 && r[best]<=1);
	
	Eigen::Matrix<Scalar, 2, 1> v;
	v(0) = (b(0)-a(0))*r[best]+a(0);
	v(1) = (b(1)-a(1))*r[best]+a(1);

	return v;
}
	
template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::subsample(const typename S::Samples &samples, const TVector &at, const Scalar r2, std::vector<Triangle> &res) const {
	//brute force (for the start)
	bool b[3];
	int n=0;
	for(int j=0; j<3; j++)
		n+= (b[j] = ( (this->p3_[j]-at).squaredNorm()<=r2))?1:0;

	if(n==0)
		return;
	else if(n==3)
		res.push_back(*this);
	else if(n==2) {
		Triangle tr1 = *this;
		Triangle tr2 = *this;
		const int ind = !b[0]?0:(!b[1]?1:2);

		/*print();
		std::cout<<"I "<<b[0]<<" "<<b[1]<<" "<<b[2]<<std::endl;
		std::cout<<"ind "<<ind<<" "<<(ind+2)%3<<std::endl;*/
		
		tr1.p_[ind] = intersection_on_line(at, r2, p_[ind], p_[ (ind+2)%3 ]);
		tr2.p_[(ind+1)%3] = intersection_on_line(at, r2, p_[ind], p_[ (ind+1)%3 ]);
		tr2.p_[ind] = tr1.p_[ind];
		
		tr1.compute(samples);
		tr2.compute(samples);
		res.push_back(tr1);
		res.push_back(tr2);
	}
	else if(n==1) {
		Triangle tr = *this;
		const int ind = b[0]?0:(b[1]?1:2);
		
		tr.p_[(ind+1)%3] = intersection_on_line(at, r2, tr.p_[ind], tr.p_[ (ind+1)%3 ]);
		tr.p_[(ind+2)%3] = intersection_on_line(at, r2, tr.p_[ind], tr.p_[ (ind+2)%3 ]);
		
		tr.compute(samples);
		res.push_back(tr);
	}
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::subsample(const TVector &at, const Scalar r2, std::vector<Triangle> &res) const {
	res.clear();
	for(size_t i=0; i<triangulated_input_.size(); i++)
		triangulated_input_[i].subsample(samples_, at, r2, res);
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::setInput(PTSurfaceList surfs) {
	input_=surfs;
	triangulated_input_.clear();
	
	for(size_t indx=0; indx<input_->size(); indx++) {
		  TPPLPartition pp;
		  list<TPPLPoly> polys,result;

		  //fill polys
		  for(size_t i=0; i<(*input_)[indx].segments_.size(); i++) {
std::cout<<"S "<<indx<<" "<<i<<" "<<(*input_)[indx].segments_[i].size()<<std::endl;
			TPPLPoly poly;
			poly.Init((*input_)[indx].segments_[i].size());
			poly.SetHole(i!=0);
			poly.SetOrientation(i!=0?TPPL_CW:TPPL_CCW);

			for(size_t j=0; j<(*input_)[indx].segments_[i].size(); j++) {
			  poly[j].x = (*input_)[indx].segments_[i][j](0);
			  poly[j].y = (*input_)[indx].segments_[i][j](1);
			}
			
			polys.push_back(poly);
			break;
		  }

		  pp.Triangulate_EC(&polys,&result);

		std::cout<<"triangles "<<result.size()<<std::endl;
		  Eigen::Vector3f v1,v2,v3;
		  for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
			if(it->GetNumPoints()!=3) continue;

			Triangle tr;
			tr.model_ = &(*input_)[indx].model_;
			for(int j=0; j<3; j++)
				Triangle::set(tr.p_[j], it->GetPoint(j));
			tr.compute(samples_);
			
			triangulated_input_.push_back(tr);
		  }
	}
}
//#define PCL_INSTANTIATE_OrganizedCurvatureEstimationOMP(T,NT,LabelT,OutT) template class PCL_EXPORTS cob_3d_features::OrganizedCurvatureEstimationOMP<T,NT,LabelT,OutT>;

