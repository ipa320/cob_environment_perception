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

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::compute() {
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
		  Eigen::FFT<Scalar> fft;
		  fft.fwd(t,f[r].abs());	//becomes translation invariant
		  (*result_)[i].ft[j].vals[r] = t.abs();	//becomes rotation invariant
	  }
	  
    }
  }
}

//transform triangle to parameterization
const Eigen::Matrix<Scalar, 3, 1> v1=at(),v2=at(),v3=at();
const Eigen::Matrix<Scalar, 3, 1> normal = ( (v2-v1).cross(v3-v1) ).normalized();
const Scalar Px = Eigen::Matrix<Scalar, 3, 1>::UnitX().dot(normal), Py = Eigen::Matrix<Scalar, 3, 1>::UnitX().dot(normal);
const Scalar P = v1(2) - (Px*v1(0) +  Py*v1(1));

template<const int Degree=1, typename Scalar>
Scalar area() {
	return 0;
}

template<const int Degree=2, typename Scalar>
Scalar area() {
	Eigen::Matrix<Scalar, 6, 1> fact;
	fact(0)=1;
	fact(1)=0.5;
	fact(2)=0.5;
	fact(3)=1/3.;
	fact(4)=0.25;
	fact(5)=1/3.;
}

template<const int Degree, typename Scalar>
Scalar area() {
	//TODO
	return 0;
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
std::complex<Scalar> cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::sub_kernel(const Scalar m, const Scalar n, const Scalar p, const Scalar x0, const Scalar y0, const Scalar d1, const Scalar d2, const Scalar e) const {
//std::cout<<m<<" "<<n<<" "<<p<<" "<<d1<<" "<<d2<<" "<<e<<" "<<x0<<" "<<y0<<std::endl;

	/*
	 * maxima eq.:
	 *   ratsimp(diff(diff(diff(integrate(integrate(integrate(%e^(-%i*(n*x+m*y+p*z)),z,p(x,y),p(x,y)+c),x,x0+(y-y0)*d1*d,x0+(y-y0)*d2*d),y,y0,y0+e1*e),c),d),e));
	 * with c=0, d=1, e=1 and e1 is e (below)
	 */
	 
	const Scalar s1 = p*model_->model(x0+e*d1,y0+e);
	const Scalar s2 = p*model_->model(x0+e*d2,y0+e);
	const Scalar s  = p*model_->model(x0,y0+e);
	
	return
		(
		  std::polar<Scalar>(d2*e*e, s1+d1*e*n) - std::polar<Scalar>(d1*e*e, s2+d2*e*n)
		) * std::polar<Scalar>(1, -(s1+s2-s + m*y0 + n*x0 + e*m + n*e*(d1+d2));
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
std::complex<Scalar> cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::kernel(const Scalar m, const Scalar n, const Scalar p) const {
	/*
		triangle -> two triangles
		    *
		   * *
		  *---*
		 *  * 
		**
		
		cut triangle in two with horizontal y-line and add them
		kernel function is computed over ordered triangles
	*/

	int indx[3] = {0,1,2};
	for(int i=0; i<2; i++)
		if(p_[indx[i]](1)>p_[indx[i+1]](1))
			std::swap(indx[i], indx[i+1]);

	const Scalar delta1=p_[indx[1]](1)-p_[indx[0]](1);
	const Scalar delta2=p_[indx[1]](1)-p_[indx[2]](1);

	const Scalar x = (p_[indx[2]](0)-p_[indx[0]](0))*(p_[indx[1]](1)-p_[indx[0]](1))/(p_[indx[2]](1)-p_[indx[0]](1));
	const Scalar left = std::min(x, p_[indx[1]](0));
	const Scalar right= std::max(x, p_[indx[1]](0));

std::cout<<left<<" "<<right<<std::endl;

	return 	(
		delta1?sub_kernel(m,n,p, p_[indx[0]](0),p_[indx[0]](1), (left-p_[indx[0]](0))/delta1, (right-p_[indx[0]](0))/delta1 ,delta1):0 -
		delta2?sub_kernel(m,n,p, p_[indx[2]](0),p_[indx[2]](1), (p_[indx[2]](0)-left)/delta2, (p_[indx[2]](0)-right)/delta2 ,delta2):0)
		;// / (); normalization?
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::compute(const std::vector<float> &radii) {
	for(int i=0; i<3; i++) {
		p3_[i] = at(p_[i]);
	}
	
	//generate now feature
	f_.resize(radii.size());
	for(size_t j=0; j<radii.size(); j++) {
		const Scalar = base = math.pow(radii[j]+1, 1./(num_radius_-1))
		
		for(int radius=0; radius<num_radius_; radius++) {
			f_[j].vals[radius].fill(0);
			const Scalar _radius = std::pow(base, radius-1)-1;
			for(int inclination=0; inclination<num_angle_; inclination++) {
				const Scalar _inclination = M_PI*inclination/num_angle_;
				for(int azimuth=0; azimuth<num_angle_; azimuth++) {
				  const Scalar _azimuth = M_PI*azimuth/num_angle_;

				const Scalar x=_radius*std::sin(_inclination)*std::cos(_azimuth);
				const Scalar y=_radius*std::sin(_inclination)*std::sin(_azimuth);
				const Scalar z=_radius*std::cos(_inclination);

				f_[j].vals[radius](inclination, azimuth) += kernel(x,y,z);
			  }
			}
		}
	}
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::generateKeypoints(std::vector<TVector> &keypoints) const {
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

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
typename cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::TVector
cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::intersection_on_line(const TVector &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b) const {
	Eigen::PolynomialSolver<Scalar, 2*TSurface::DEGREE+1> solver;	
	typename TSurface::Model::VectorU1D p = model_->transformation_1D(a-b,a, at);
	p(0) -= r2;
	solver.compute(p);
	
	std::vector<Scalar> r;
	solver.realRoots(r);
	assert(r.size()>0);
	
	const float mid = 0.5f;
	size_t best=0;
	for(size_t i=1;i<r.size();++i)
	{
		if( (r[best]-mid)*(r[best]-mid)>(r[i]-mid)*(r[i]-mid))
			best=i;
	}
	
	TVector v;
	v(0) = (a(0)-b(0))*r[best]+a(0);
	v(1) = (a(1)-b(1))*r[best]+a(1);
	v(2) = model_->model( v(0), v(1) );
	
	return v;
}
	
template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::subsample(const TVector &at, const Scalar r2, std::vector<Triangle> &res) {
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
			Triangle tr1 = triangulated_input_[i];
			Triangle tr2 = triangulated_input_[i];
			const int ind = !b[0]?0:(!b[1]?1:2);
			
			tr1.p_[ind] = intersection_on_line(at, r2, tr1.p_[ind], tr1.p_[ (ind+2)%3 ]);
			tr2.p_[(ind+1)%3] = intersection_on_line(at, r2, tr2.p_[ind], tr2.p_[ (ind+1)%3 ]);
			tr2.p_[ind] = tr1.p_[ind];
			
			tr1.compute();
			tr2.compute();
			res.push_back(tr1);
			res.push_back(tr2);
		}
		else if(n==1) {
			Triangle tr = triangulated_input_[i];
			const int ind = b[0]?0:(b[1]?1:2);
			
			tr.p_[(ind+1)%3] = intersection_on_line(at, r2, tr.p_[ind], tr.p_[ (ind+1)%3 ]);
			tr.p_[(ind+2)%3] = intersection_on_line(at, r2, tr.p_[ind], tr.p_[ (ind+2)%3 ]);
			
			tr.compute();
			res.push_back(tr);
		}
	}
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::setInput(PTSurfaceList surfs) {
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
				Triangle::set(tr.t_[j], it->GetPoint(j));
			tr.compute();
			
			triangulated_input_.push_back(tr);
		  }
	}
}
//#define PCL_INSTANTIATE_OrganizedCurvatureEstimationOMP(T,NT,LabelT,OutT) template class PCL_EXPORTS cob_3d_features::OrganizedCurvatureEstimationOMP<T,NT,LabelT,OutT>;

