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
 * ROS package name: cob_3d_mapping_features
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

#include "cob_3d_mapping_features/invariant_surface_feature.h"
#include <unsupported/Eigen/FFT>
#include <unsupported/Eigen/Polynomials>

void cob_3d_mapping_features::InvariantSurfaceFeature<>::compute() {
  result_.reset(new ResultVector);

  //generate keypoints (e.g. reduce number of points by area)
  std::vector<TVector> keypoints;
  generateKeypoints(keypoints);

  result_->resize(keypoints.size());
  for(size_t i=0; i<keypoints.size(); i++) {
    (*result_)[i].v = keypoints[i];
    (*result_)[i].ft.resize(radii_.size());
    for(size_t j=0; j<radii_.size(); j++) {
		
      //generate sub map
      std::vector<Triangle> submap;
      subsample(keypoints[i], radii_[j], submap);

	  //sum features
	  FeatureComplex f; f.fill(0);
	  for(size_t s=0; s<submap.size(); s++)
		  f += submap[s].f_;

	  //calc. rotation invariant feature
	  for(int r=0; r<num_radius_; r++) {
		  FeatureAngleComplex t;
		  FFT<Scalar> fft;
		  fft.fwd(t,f[r].abs());	//becomes translation invariant
		  (*result_)[i].ft[j][r] = t.abs();	//becomes rotation invariant
	  }
	  
    }
  }
}
		
void cob_3d_mapping_features::InvariantSurfaceFeature<>::Triangle::compute() {
	for(int i=0; i<3; i++) {
		p3_[i](0) = p_[i](0);
		p3_[i](1) = p_[i](1);
		p3_[i](2) = model_->model(p_[i](0), p_[i](1));
	}
	
	//generate now feature
	f_.fill(0);
	for(int radius=0; radius<num_radius_; radius++) {
		Scalar _radius = radius*radii_[j]/num_radius_;
		for(int inclination=0; inclination<num_angle_; inclination++) {
			Scalar _inclination = M_PI*inclination/num_angle_;
			for(int azimuth=0; azimuth<num_angle_; azimuth++) {
			  Scalar _azimuth = M_PI*azimuth/num_angle_;

			/*
			 * x=_radius*std::sin(_inclination)*std::cos(_azimuth);
			 * y=_radius*std::sin(_inclination)*std::sin(_azimuth);
			 * z=_radius*std::cos(_inclination);
			 */
			f_[radius](inclination, azimuth) += kernel();
		  }
		}
	}
}

void cob_3d_mapping_features::InvariantSurfaceFeature<>::generateKeypoints(std::vector<TVector> &keypoints) {
  //TODO: reduce points

  for(size_t i=0; i<input_->size(); i++) {                         //surfaces
    for(size_t j=0; j<(*input_)[i].segments_.size(); j++) {        //outline/holes
      for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++) {   //points
        TVector v;
        v(0) = (*input_)[i].segments_[j][k](0);
        v(1) = (*input_)[i].segments_[j][k](1);
        v(2) = (*input_)[i].model_.model( v(0), v(1) );
        keypoints.push_back(v);
      }
    }
  }

}

TVector cob_3d_mapping_features::InvariantSurfaceFeature<>::subsample(const TVector &at, const Scalar r2, const Ei) {
	Eigen::PolynomialSolver<Scalar, DegreeN> solver;	
	p = model->transform;
	p(1) += mx*mx;
	p(2) += my*my;
	solver.compute(p);
	std::vector<Scalar> r;
	solver.realRoots(r);
	
	assert(r.size()>0);
	
	size_t b=0;
	for(size_t i=1;i<r.size();++i)
	{
		if( (r[b]-mid)*(r[b]-mid)>(r[i]-mid)*(r[i]-mid))
			b=i;
	}
	
	TVector v;
	v(0) = ;
	v(1) = ;
	v(2) = ;
	
	return v;
}
	
void cob_3d_mapping_features::InvariantSurfaceFeature<>::subsample(const TVector &at, const Scalar r2, std::vector<Triangle> &res) {
	//brute force (for the start)
	for(size_t i=0; i<triangulated_input_.size(); i++) {
		bool b[3];
		int n=0;
		for(int j=0; j<3; j++)
			n+= (b[j] = ( (triangulated_input_[i].p3_[j]-at).squaredNorm()<=r2))?1:0;
			
		if(n==0)
			continue;
		else if(n==3)
			res.push_back(triangulated_input_[i]);
		else if(n==2) {
			Triangle tr = triangulated_input_[i];
			//TODO:
			tr.compute();
		}
		else if(n==1) {
			Triangle tr = triangulated_input_[i];
			//TODO:
			tr.compute();
		}
	}
}

void cob_3d_mapping_features::InvariantSurfaceFeature<>::setInput(PTSurfaceList surfs) {
	input_=surfs;
	triangulated_input_.clear();
	
	for(size_t indx=0; indx<input_->size(); indx++) {
		  TPPLPartition pp;
		  list<TPPLPoly> polys,result;

		  //fill polys
		  for(size_t i=0; i<(*input_)[indx].segments_.size(); i++) {
			TPPLPoly poly;
			poly.Init((*input_)[indx].segments_[i].size());
			poly.SetHole(i!=0);
			poly.SetOrientation(i!=0?TPPL_CW:TPPL_CCW);

			for(size_t j=0; j<(*input_)[indx].segments_[i].size(); j++) {
			  poly[j].x = (*input_)[indx].segments_[i][j](0);
			  poly[j].y = (*input_)[indx].segments_[i][j](1);
			}
			
			polys.push_back(poly);
		  }

		  pp.Triangulate_EC(&polys,&result);

		  Eigen::Vector3f v1,v2,v3;
		  for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
			if(it->GetNumPoints()!=3) continue;

			Triangle tr;
			tr.model_ = &(*input_)[indx].model_;
			for(int j=0; j<3; j++)
				Triangle::set(tr_.t_[j], it->GetPoint(j));
			tr.compute();
			
			triangulated_input_.push_back(tr);
		  }
	}
}
//#define PCL_INSTANTIATE_OrganizedCurvatureEstimationOMP(T,NT,LabelT,OutT) template class PCL_EXPORTS cob_3d_mapping_features::OrganizedCurvatureEstimationOMP<T,NT,LabelT,OutT>;

