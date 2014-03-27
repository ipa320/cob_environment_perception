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

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
std::complex<Scalar> cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::kernel_lin(const Scalar m, const Scalar n, const Scalar p, const Scalar x0, const Scalar y0, const Scalar y1, const Scalar d1, const Scalar d2) const {
	//Eigen::Matrix<Scalar, 3, 1>
	const Scalar Px=0, Py=0, P=0;

	/*MAXIMA:
	p(x,y):=Px*x+Py*y+P;
	ratsimp(diff(diff(diff(integrate(integrate(integrate(%e^(-%i*(n*x+m*y+p*z)),z,p(x,y),p(x,y)+c),x,x0,x0+d),y,y0,y0+e),e),c),d));
	ff(c,d,e,x0,y0):=%e^(-%i*p*P-%i*p*Py*y0-%i*m*y0-%i*p*Px*x0-%i*n*x0-%i*e*p*Py-%i*d*p*Px-%i*c*p-%i*d*n-%i*e*m);
	ratsimp(integrate(integrate(ff(0,0,0,x,y),x,x0+(y-y0)*d1,x0+(y-y0)*d2),y,y0,y1));

	result:
	-((((d2-d1)*p*Px+(d2-d1)*n)*%e^(%i*p*Py*y1+%i*d2*p*Px*y1+%i*d1*p*Px*y1+%i*d2*n*y1+%i*d1*n*y1+%i*m*y1)+(-p*Py-d2*p*Px-d2*n-m)*
	%e^(%i*d2*p*Px*y1+%i*d2*n*y1+%i*p*Py*y0+%i*d1*p*Px*y0+%i*d1*n*y0+%i*m*y0)+(p*Py+d1*p*Px+d1*n+m)*%e^(%i*d1*p*Px*y1+%i*d1*n*y1+%i*p*Py*y0+%i*d2*p*Px*y0+%i*d2*n*y0+%i*m*y0))*
	%e^(-%i*p*P-%i*p*Py*y1-%i*d2*p*Px*y1-%i*d1*p*Px*y1-%i*d2*n*y1-%i*d1*n*y1-%i*m*y1-%i*p*Py*y0-%i*m*y0-%i*p*Px*x0-%i*n*x0))/((p^3*Px+n*p^2)*Py^2+
	((d2+d1)*p^3*Px^2+((2*d2+2*d1)*n+2*m)*p^2*Px+((d2+d1)*n^2+2*m*n)*p)*Py+d1*d2*p^3*Px^3+(3*d1*d2*n+(d2+d1)*m)*p^2*Px^2+(3*d1*d2*n^2+(2*d2+2*d1)*m*n+m^2)*p*Px+d1*d2*n^3+
	(d2+d1)*m*n^2+m^2*n)
	*/

	if(m==0 && n==0 && p==0) {
		return ((d2-d1)*std::pow(y1-y0,2))/2;
	}

	const std::complex<Scalar> tt = std::polar<Scalar>(1, -( p*P+(p*Py+(d2+d1)*p*Px+(d2+d1)*n+m)*y1+(p*Py+m)*y0+(p*Px+n)*x0 ));

	const std::complex<Scalar> t1 = std::polar<Scalar>( (d2-d1)*(p*Px+n), 		(p*Py+(d2+d1)*p*Px+(d2+d1)*n+m)*y1);
	const std::complex<Scalar> t2 = std::polar<Scalar>( -(p*Py+d2*p*Px+d2*n+m), 	(d2*p*Px+d2*n)*y1+(p*Py+d1*p*Px+d1*n+m)*y0);
	const std::complex<Scalar> t3 = std::polar<Scalar>( p*Py+d1*p*Px+d1*n+m,	(d1*p*Px+d1*n)*y1+(p*Py+d2*p*Px+d2*n+m)*y0);

	const Scalar div = (p*Px+n)*(p*Py+d1*p*Px+d1*n+m)*(p*Py+d2*p*Px+d2*n+m);

	return  -((t1+t2+t3)*tt)/div;
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
std::complex<Scalar> cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::kernel_lin_tri(const Scalar m, const Scalar n, const Scalar p, const Tri2D &tri) const {
	int indx[3] = {0,1,2};
	for(int i=0; i<2; i++)
		if((*tri.p_[indx[i]])(1)>(*tri.p_[indx[i+1]])(1))
			std::swap(indx[i], indx[i+1]);

	const Scalar delta1=(*tri.p_[indx[1]])(1)-(*tri.p_[indx[0]])(1);
	const Scalar delta2=(*tri.p_[indx[1]])(1)-(*tri.p_[indx[2]])(1);

	const Scalar x = ((*tri.p_[indx[2]])(0)-(*tri.p_[indx[0]])(0))*((*tri.p_[indx[1]])(1)-(*tri.p_[indx[0]])(1))/((*tri.p_[indx[2]])(1)-(*tri.p_[indx[0]])(1));
	const Scalar left = std::min(x, (*tri.p_[indx[1]])(0));
	const Scalar right= std::max(x, (*tri.p_[indx[1]])(0));

std::cout<<left<<" "<<right<<std::endl;

	return 	(
		delta1?kernel_lin(m,n,p, (*tri.p_[indx[0]])(0),(*tri.p_[indx[0]])(1),(*tri.p_[indx[1]])(1), (left-(*tri.p_[indx[0]])(0))/delta1, (right-(*tri.p_[indx[0]])(0))/delta1):0 -
		delta2?kernel_lin(m,n,p, (*tri.p_[indx[2]])(0),(*tri.p_[indx[2]])(1),(*tri.p_[indx[1]])(1), ((*tri.p_[indx[2]])(0)-left)/delta2, ((*tri.p_[indx[2]])(0)-right)/delta2):0)
		;// / (); normalization?
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
std::complex<Scalar> cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::sub_kernel(const Scalar m, const Scalar n, const Scalar p, const Tri2D &tri) const {
	//check if further sub-sampling is necessary?
	if(area<TSurface::DEGREE>(tri)>0.05) {
		Eigen::Matrix<Scalar, 2, 1> ps[3];
		for(int i=0; i<3; i++)
			ps[i] = ((*tri.p_[i])+(*tri.p_[(i+1)%3]))/2;

		Tri2D tris[3]={tri, tri, tri};
		for(int i=0; i<3; i++) {
			tris[i].p_[(i+1)%3] = &ps[i];
			tris[i].p_[(i+2)%3] = &ps[(i+2)%3];
		}

		return kernel_lin_tri(m,n,p, tris[0])+kernel_lin_tri(m,n,p, tris[1])+kernel_lin_tri(m,n,p, tris[2]);
	} else 
		return kernel_lin_tri(m,n,p, tri);
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

	const Tri2D tri = {&p_[0],&p_[1],&p_[2]};
	return sub_kernel(m,n,p, tri);
}

template<const int num_radius_, const int num_angle_, typename TSurface, typename Scalar, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<num_radius_,num_angle_,TSurface,Scalar,TAffine>::Triangle::compute(const std::vector<float> &radii) {
	for(int i=0; i<3; i++) {
		p3_[i](0) = p_[i](0);
		p3_[i](1) = p_[i](1);
		p3_[i](2) = model_->model(p_[i](0), p_[i](1));
	}
	
	//generate now feature
	f_.resize(radii.size());
	for(size_t j=0; j<radii.size(); j++) {
		const Scalar base = std::pow(radii[j]+1, 1./(num_radius_-1));
		
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

