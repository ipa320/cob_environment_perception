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

#define NUM_THREADS 1

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::compute() {
  result_.reset(new Result);

  for(size_t i=0; i<keypoints_.size(); i++)  {
	result_->push_back( Feature(keypoints_[i]) );
	result_->back().f_.resize(radii_.size());
  }
	
	//static int dbg_stage=0, dbg_num=0;
	//++dbg_stage;
	
  #pragma omp parallel for num_threads( NUM_THREADS )
  for(size_t i=0; i<keypoints_.size(); i++) {		
    std::vector<boost::shared_ptr<Triangle> > submap_last;
    for(size_t j=0; j<radii_.size(); j++) {
      //generate sub map
      std::vector<boost::shared_ptr<Triangle> > submap;
      subsample(keypoints_[i], radii_[j], submap, j==0?triangulated_input_:submap_last);
      
      if(j+1<radii_.size())
		submap_last = submap;
      
      /*char dbg_fn[128];
      sprintf(dbg_fn,"/tmp/submap%d_%d_%d.ply", (int)i, (int)j, dbg_stage);
	  pcl::io::savePLYFile(dbg_fn, *dbg_triangles2mesh(submap));
	  //std::cout<<"submap triangles: "<<submap.size()<<std::endl;
	  dbg_num += submap.size();*/

      #pragma omp critical
      {
		  //calc. rotation invariant feature
		  sr_.clear();
		  //sum features
		  //(*result_)[i].area_ = 0;
		  for(size_t s=0; s<submap.size(); s++) {
			  sr_ += *submap[s];
			  (*result_)[i].area_ += submap[s]->getArea();
		  }
			  
		  sr_.finish((*result_)[i].f_[j]);
		  (*result_)[i].normalize(radii_);
	  }
    }
  }
  
	//std::cout<<"sum triangles: "<<dbg_num<<std::endl;
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::compute(const typename S::Samples &samples) {		
	if(this->cr<0 || !this->computed_) {	//if not already set
		for(int i=0; i<3; i++)
			this->p3_[i] = (*descr_)[p_[i]];
	
		this->d1 = this->p3_[2]-this->p3_[1];
		this->d2 = this->p3_[1]-this->p3_[0];
		this->cr = this->d1.cross(this->d2).norm();
	}
	
	//check if further sub-sampling is necessary?
	if(area()>0.0001/*||rand()%13==0||depth<4*/) {
//std::cout<<"subdivide "<<area<TSurface::DEGREE>()<<std::endl;
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
		
		this->f_.resize(samples.size());
		for(size_t r=0; r<samples.size(); r++)
			this->f_[r].resize(samples[r].size(), 0);
			
		for(int i=0; i<4; i++) {
			tris[i].descr_ = descr_;
			tris[i].compute(samples);
			for(size_t j=0; j<samples.size(); j++)
				for(size_t k=0; k<samples[j].size(); k++)
					this->f_[j][k] += tris[i].values()[j][k];
		}
		this->computed_ = true;
	} else 
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, typename S::Samples, typename S::Values>::compute(samples);
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::generateKeypoints(std::vector<TVector> &keypoints) const {
  //reduce points by area weightning

  for(size_t i=0; i<input_->size(); i++)                         //surfaces
	(*input_)[i].template generateKeypoints<TVector,Real>(keypoints, kp_min_area_, kp_area_);
	
	ROS_INFO("keypoints %zu",keypoints.size());
}

template<typename Shape>
template<typename TVector, typename Real>
void cob_3d_features::TriangleSurfaceHelper<Shape>::generateKeypoints(std::vector<TVector> &keypoints, const Real kp_min_area_, const Real kp_area_) {
	typedef typename TVector::Scalar Scalar;
	
    for(size_t j=0; j<contours_.size(); j++) {        //outline/holes
    
      //std::cout<<"size bef "<<contours_[j].size()<<std::endl;
    
	  typedef std::list<Eigen::Matrix<Real, 2, 1> > LIST;
	  typedef std::list<Real> LIST2;
	  LIST pts;
	  LIST2 pts_area;
	  float area = 0.f, min_area;
	  const Real FACTOR = 0.05f;
	  const Real FACTOR_MIN = 0.0025f;
	  
      TVector avg_pt;
      avg_pt.fill(0);
      for(size_t k=0; k<contours_[j].size(); k++)   //points
      {
		  Eigen::Matrix<Real, 2, 2> M;
		  Eigen::Matrix<Real, 2, 1> pt2d = contours_[j][k ].head(2);
		  M.col(0) = pt2d;
		  M.col(1) = contours_[j][(k+1)%contours_[j].size()].head(2);
		  area -= M.determinant()/2;

		  avg_pt += (*this)[pt2d].template cast<typename TVector::Scalar>();
		  
		  pts.push_back(pt2d);
		  pts_area.push_back(0);
	  }
	  avg_pt /= contours_[j].size();
	  //assert(area>=0);
	  area = std::abs(area);
	  
	  min_area = std::max(area*FACTOR_MIN, kp_min_area_*std::max((Real)1, (Real)avg_pt.squaredNorm())); //TODO: set threshold [cm^2]
	  area*=FACTOR;
	  area = std::max(area, kp_area_);	//TODO: set threshold [cm^2]
	  
	  while(pts.size()>=3) {
		  bool rem = false;	//if nothing was removed, calc. remaining area
		  float mi = std::numeric_limits<float>::max();	//minimum weight
		  typename LIST::iterator mit = pts.end();	//iterator to minim weighted pt.
		  typename LIST2::iterator mit2_new = pts_area.end(), mit2_old = pts_area.end();
		  //float remaining_area = 0.f;	//if remaining area is already smaller than threshold --> break
		  
		  /*static int nnn=0;
		  std::ofstream of( ("/tmp/poly"+boost::lexical_cast<std::string>(nnn++)+".svg").c_str() );
		  of<<"<svg height=\"500\" width=\"500\"><polygon points=\"";
		  for(typename LIST::iterator it=pts.begin(); it!=pts.end(); it++) {
			  of<<(*it)(0)*300+200<<","<<(*it)(1)*300+200<<" ";
		  }
		  of<<"\" /></svg>";
		  of.close();*/
		  
		  typename LIST2::iterator it2=pts_area.begin();
		  for(typename LIST::iterator it=pts.begin(); it!=pts.end(); it++, it2++) {
			  typename LIST::iterator n1 = it, n2=it;
			  n1++; n2++; if(n1==pts.end()) n1=n2=pts.begin();
			  n2++; if(n2==pts.end()) n2=pts.begin();
			  typename LIST2::iterator an1 = it2;
			  an1++; if(an1==pts_area.end()) an1=pts_area.begin();
			  
			  Eigen::Matrix<Real, 2, 2> M;
			  M.col(0) = (*it-*n1);
			  M.col(1) = (*n2-*n1);
			  const float w = std::abs(M.determinant()/2) + *it2 + *an1;
			  //std::cout<<"w "<<w<<std::endl;
			  
			  /*if(!rem) {
				  M.col(0) = *it;
				  M.col(1) = *n1;
				  remaining_area-=M.determinant()/2;
			  }*/
			  
			  /*if(w>=area) {	//fulfills already our criteria
			  //std::cout<<"rem"<<std::endl;
				  keypoints.push_back(this->at(*n1, model_));
				  pts.erase(n1);
				  it--;
				  rem = true;
			  } else*/ if(w<mi) {
				mit = n1;
				mit2_new = it2;
				mit2_old = an1;
				mi = w;
			}
		  }
		  
		  if(mi>area) {	//fulfills already our criteria
			  //std::cout<<"add keypoints "<<pts.size()<<std::endl;
			  for(typename LIST::const_iterator pit=pts.begin(); pit!=pts.end(); pit++) keypoints.push_back((*this)[*pit].template cast<typename TVector::Scalar>());
			  break;
		  }
		  
		  //if(rem) continue;
			  //std::cout<<"s "<<pts.size()<<" "<<(mit_n)<<std::endl;
		  //if( (!rem && remaining_area<area) || pts.size()<3) break;
		  assert(mit!=pts.end());
		  
		  if(mi<min_area) {
			  //TODO: improve
			  for(size_t k=0; k<contours_[j].size(); k++)   //points
				if(contours_[j][k].head(2)==*mit) {
					contours_[j].erase(contours_[j].begin()+k);
					//std::cout<<"reduced by one point"<<std::endl;
					break;
				}
		  }
		  pts.erase(mit);	//remove point with minimum weight --> increases weight of surroundings
		  *mit2_new += mi;
		  pts_area.erase(mit2_old);
	  }
	  
      //std::cout<<"size aft "<<contours_[j].size()<<std::endl;
	}
}

template<typename PSurface>
template<typename TVector, typename Real>
void cob_3d_features::PolynomialSurfaceHelper<PSurface>::generateKeypoints(std::vector<TVector> &keypoints, const Real kp_min_area_, const Real kp_area_) {
	typedef typename TVector::Scalar Scalar;
	
    for(size_t j=0; j<segments_.size(); j++) {        //outline/holes
    
      //std::cout<<"size bef "<<segments_[j].size()<<std::endl;
    
	  typedef std::list<Eigen::Matrix<Real, 2, 1> > LIST;
	  typedef std::list<Real> LIST2;
	  LIST pts;
	  LIST2 pts_area;
	  float area = 0.f, min_area;
	  const Real FACTOR = 0.05f;
	  const Real FACTOR_MIN = 0.0025f;
	  
      TVector avg_pt;
      avg_pt.fill(0);
      for(size_t k=0; k<segments_[j].size(); k++)   //points
      {
		  Eigen::Matrix<Real, 2, 2> M;
		  Eigen::Matrix<Real, 2, 1> pt2d = segments_[j][k ].head(2);
		  M.col(0) = pt2d;
		  M.col(1) = segments_[j][(k+1)%segments_[j].size()].head(2);
		  area -= M.determinant()/2;

		  avg_pt += (*this)[pt2d].template cast<typename TVector::Scalar>();
		  
		  pts.push_back(pt2d);
		  pts_area.push_back(0);
	  }
	  avg_pt /= segments_[j].size();
	  //assert(area>=0);
	  area = std::abs(area);
	  
	  min_area = std::max(area*FACTOR_MIN, kp_min_area_*std::max((Real)1, (Real)avg_pt.squaredNorm())); //TODO: set threshold [cm^2]
	  area*=FACTOR;
	  area = std::max(area, kp_area_);	//TODO: set threshold [cm^2]
	  
	  while(pts.size()>=3) {
		  bool rem = false;	//if nothing was removed, calc. remaining area
		  float mi = std::numeric_limits<float>::max();	//minimum weight
		  typename LIST::iterator mit = pts.end();	//iterator to minim weighted pt.
		  typename LIST2::iterator mit2_new = pts_area.end(), mit2_old = pts_area.end();
		  //float remaining_area = 0.f;	//if remaining area is already smaller than threshold --> break
		  
		  /*static int nnn=0;
		  std::ofstream of( ("/tmp/poly"+boost::lexical_cast<std::string>(nnn++)+".svg").c_str() );
		  of<<"<svg height=\"500\" width=\"500\"><polygon points=\"";
		  for(typename LIST::iterator it=pts.begin(); it!=pts.end(); it++) {
			  of<<(*it)(0)*300+200<<","<<(*it)(1)*300+200<<" ";
		  }
		  of<<"\" /></svg>";
		  of.close();*/
		  
		  typename LIST2::iterator it2=pts_area.begin();
		  for(typename LIST::iterator it=pts.begin(); it!=pts.end(); it++, it2++) {
			  typename LIST::iterator n1 = it, n2=it;
			  n1++; n2++; if(n1==pts.end()) n1=n2=pts.begin();
			  n2++; if(n2==pts.end()) n2=pts.begin();
			  typename LIST2::iterator an1 = it2;
			  an1++; if(an1==pts_area.end()) an1=pts_area.begin();
			  
			  Eigen::Matrix<Real, 2, 2> M;
			  M.col(0) = (*it-*n1);
			  M.col(1) = (*n2-*n1);
			  const float w = std::abs(M.determinant()/2) + *it2 + *an1;
			  //std::cout<<"w "<<w<<std::endl;
			  
			  /*if(!rem) {
				  M.col(0) = *it;
				  M.col(1) = *n1;
				  remaining_area-=M.determinant()/2;
			  }*/
			  
			  /*if(w>=area) {	//fulfills already our criteria
			  //std::cout<<"rem"<<std::endl;
				  keypoints.push_back(this->at(*n1, model_));
				  pts.erase(n1);
				  it--;
				  rem = true;
			  } else*/ if(w<mi) {
				mit = n1;
				mit2_new = it2;
				mit2_old = an1;
				mi = w;
			}
		  }
		  
		  if(mi>area) {	//fulfills already our criteria
			  //std::cout<<"add keypoints "<<pts.size()<<std::endl;
			  for(typename LIST::const_iterator pit=pts.begin(); pit!=pts.end(); pit++) keypoints.push_back((*this)[*pit].template cast<typename TVector::Scalar>());
			  break;
		  }
		  
		  //if(rem) continue;
			  //std::cout<<"s "<<pts.size()<<" "<<(mit_n)<<std::endl;
		  //if( (!rem && remaining_area<area) || pts.size()<3) break;
		  assert(mit!=pts.end());
		  
		  if(mi<min_area) {
			  //TODO: improve
			  for(size_t k=0; k<segments_[j].size(); k++)   //points
				if(segments_[j][k].head(2)==*mit) {
					segments_[j].erase(segments_[j].begin()+k);
					//std::cout<<"reduced by one point"<<std::endl;
					break;
				}
		  }
		  pts.erase(mit);	//remove point with minimum weight --> increases weight of surroundings
		  *mit2_new += mi;
		  pts_area.erase(mit2_old);
	  }
	  
      //std::cout<<"size aft "<<segments_[j].size()<<std::endl;
	}
}

template<typename Shape>
template<typename Scalar>
Eigen::Matrix<Scalar, 2, 1>
cob_3d_features::TriangleSurfaceHelper<Shape>::intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, bool &success) const {

//assert((this->at(a)-at).squaredNorm()>r2 || (this->at(b)-at).squaredNorm()>r2);

	const Eigen::Matrix<Scalar, 3, 1> p1 = (*this)[b], p2 = (*this)[a];
	const Eigen::Matrix<Scalar, 3, 1> v = p1-p2, t = p2-at;
	
	const Scalar ma = v.squaredNorm(), mb = 2*v.dot(t), mc = t.squaredNorm()-r2;
	
	success = (mb*mb-4*ma*mc>0);
	
	Scalar mr1 = (-mb+std::sqrt(mb*mb-4*ma*mc))/(2*ma);
	const Scalar mr2 = (-mb-std::sqrt(mb*mb-4*ma*mc))/(2*ma);
	
	const Scalar mid = 0.5f;
	if( std::abs(mr1-mid)>std::abs(mr2-mid))
		mr1 = mr2;

	if(!(mr1>=0 && mr1<=1)) std::cout<<mr1<<std::endl;
	//assert(mr1>=-0.0001 && mr1<=1.0001);
	success &= (mr1>0 && mr1<1);

	return (b-a)*mr1+a;
}

template<typename Shape>
template<typename Scalar>
bool cob_3d_features::TriangleSurfaceHelper<Shape>::intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, Eigen::Matrix<Scalar, 2, 1> &res1, Eigen::Matrix<Scalar, 2, 1> &res2, bool &res_b1, bool &res_b2) const {
	const Eigen::Matrix<Scalar, 3, 1> p1 = (*this)[b], p2 = (*this)[a];
	const Eigen::Matrix<Scalar, 3, 1> v = p1-p2, t = p2-at;
	
	const Scalar ma = v.squaredNorm(), mb = 2*v.dot(t), mc = t.squaredNorm()-r2;
	if(mb*mb-4*ma*mc<=0) return false;
	
	Scalar mr2 = (-mb+std::sqrt(mb*mb-4*ma*mc))/(2*ma);
	Scalar mr1 = (-mb-std::sqrt(mb*mb-4*ma*mc))/(2*ma);
	
	if( (mr1<=0&&mr2<=0) || (mr1>=1&&mr2>=1) )
		return false;
	
	if(mr1<0) {mr1 = 0;res_b1=true;} else res_b1=false;
	if(mr2>1) {mr2 = 1;res_b2=true;} else res_b2=false;
	
	//std::cout<<r[best1]<<" "<<r[best2]<<std::endl;
	res1 = (b-a)*mr1+a;
	res2 = (b-a)*mr2+a;

	return true;
}

template<typename PSurface>
template<typename Scalar>
Eigen::Matrix<Scalar, 2, 1>
cob_3d_features::PolynomialSurfaceHelper<PSurface>::intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, bool &success) const {

//assert((this->at(a)-at).squaredNorm()>r2 || (this->at(b)-at).squaredNorm()>r2);

	Eigen::PolynomialSolver<typename PSurface::Model::VectorU1D::Scalar, 2*PSurface::DEGREE> solver;	
	typename PSurface::Model::VectorU1D p = model_.transformation_1D( (b-a).template cast<typename PSurface::Model::VectorU1D::Scalar>(), a.template cast<typename PSurface::Model::VectorU1D::Scalar>(), at.template cast<typename PSurface::Model::VectorU1D::Scalar>());
	p(0) -= r2;
	solver.compute(p);
	
	std::vector<typename PSurface::Model::VectorU1D::Scalar> r;
	solver.realRoots(r);
	assert(r.size()>0);
	
	const typename PSurface::Model::VectorU1D::Scalar mid = 0.5f;
	size_t best=0;
	for(size_t i=1;i<r.size();++i)
	{
		if( std::abs(r[best]-mid)>std::abs(r[i]-mid))
			best=i;
	}

	if(!(r[best]>=0 && r[best]<=1)) std::cout<<r[best]<<std::endl;
	//assert(r[best]>=-0.0001 && r[best]<=1.0001);
	success = (r[best]>0 && r[best]<1);
	
	Eigen::Matrix<Scalar, 2, 1> v;
	v(0) = (b(0)-a(0))*r[best]+a(0);
	v(1) = (b(1)-a(1))*r[best]+a(1);

	return v;
}

template<typename PSurface>
template<typename Scalar>
bool cob_3d_features::PolynomialSurfaceHelper<PSurface>::intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, Eigen::Matrix<Scalar, 2, 1> &res1, Eigen::Matrix<Scalar, 2, 1> &res2, bool &res_b1, bool &res_b2) const {
	Eigen::PolynomialSolver<typename PSurface::Model::VectorU1D::Scalar, 2*PSurface::DEGREE> solver;	
	typename PSurface::Model::VectorU1D p = model_.transformation_1D( (b-a).template cast<typename PSurface::Model::VectorU1D::Scalar>(), a.template cast<typename PSurface::Model::VectorU1D::Scalar>(), at.template cast<typename PSurface::Model::VectorU1D::Scalar>());
	p(0) -= r2;
	solver.compute(p);
	
	std::vector<typename PSurface::Model::VectorU1D::Scalar> r;
	solver.realRoots(r);
	if(r.size()<2) return false;
	
	size_t best1=0, best2=0;
	for(size_t i=1;i<r.size();++i)
	{
		if( r[best1]>r[i] )//&& std::abs(r[i])>0)
			best1=i;
		if( r[best2]<r[i] )//&& std::abs(r[i])<1)
			best2=i;
	}
	
	if( (r[best1]<=0&&r[best2]<=0) || (r[best1]>=1&&r[best2]>=1) )
		return false;
	
	if(r[best1]<0) {r[best1] = 0;res_b1=true;} else res_b1=false;
	if(r[best2]>1) {r[best2] = 1;res_b2=true;} else res_b2=false;
	
	//std::cout<<r[best1]<<" "<<r[best2]<<std::endl;
	res1(0) = (b(0)-a(0))*r[best1]+a(0);
	res1(1) = (b(1)-a(1))*r[best1]+a(1);
	
	res2(0) = (b(0)-a(0))*r[best2]+a(0);
	res2(1) = (b(1)-a(1))*r[best2]+a(1);

	return true;
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::subsample(const boost::shared_ptr<Triangle> &_this, const typename S::Samples &samples, const TVector &at, const Scalar r2, std::vector<boost::shared_ptr<Triangle> > &res) const {
	
	if(0) {
		int n=0;
		for(int j=0; j<3; j++) {
			n+= (( (this->p3_[j]-at).squaredNorm()<=r2))?1:0;
		}
		if(n) {
			res.push_back(_this);
			return;
		}
		return;
	}
	
	std::vector<Eigen::Matrix<Scalar, 2, 1> > pts, pts_opp;
	std::vector<bool> pts_ins;
	
	int n=-1;
	for(int j=0; j<3; j++) {
		bool rb1, rb2;
		Eigen::Matrix<Scalar, 2, 1> rv1,rv2;
		if(descr_->intersection_on_line(at, r2, p_[j], p_[ (j+1)%3 ], rv1,rv2, rb1, rb2)) {
			pts.push_back(rv1);
			pts_ins.push_back(rb1);
			
			pts.push_back(rv2);
			pts_ins.push_back(rb2);
			
			pts_opp.push_back(p_[ (j+2)%3 ]);
			pts_opp.push_back(p_[ (j+2)%3 ]);
			
			n=j;
		}
	}
	
	size_t p=0;
	
	if(pts.size()<2)
		return;
		
	else if(pts.size()==2) {
		bool success;
		Eigen::Matrix<Scalar, 2, 1> t=descr_->template intersection_on_line<Scalar>(at, r2, (pts.front()+pts.back())/2, p_[ (n+2)%3 ], success);
		if(success) {
			pts.push_back(t);
			pts_ins.push_back(false);
			
			pts_opp.push_back(p_[ (n+1)%3 ]);
		}
		//assert(success);
		++p;
	}
	
	//subsample further
	int num=0;
	//std::cout<<"bef "<<pts.size()<<std::endl;
	while(num<8 && p<pts.size()&&1) {
		if(pts_ins[p] || pts_ins[(p+1)%pts.size()]) {
			++p;
			continue;
		}
		const Scalar er = std::abs( (((*descr_)[(Eigen::Matrix<Scalar, 2, 1>)( (pts[p]+pts[(p+1)%pts.size()])/2 )] - at).squaredNorm()/r2)-1 );
	//std::cout<<"er "<<er<<std::endl;
		if( er > 0.02) { //TODO: set threshold
			bool success;
			Eigen::Matrix<Scalar, 2, 1> s = pts_opp[p], m = (pts[p]+pts[(p+1)%pts.size()])/2;
			Eigen::Matrix<Scalar, 2, 1> v = descr_->template intersection_on_line<Scalar>(at, r2, (m-s)*2+s, s, success);
			if(success) {
				++num;
				pts.insert(pts.begin()+(p+1), v);
				pts_ins.insert(pts_ins.begin()+(p+1), false);
				pts_opp.insert(pts_opp.begin()+(p+1),pts_opp[p]);
			} else ++p;
		}
		else ++p;
	}
	//std::cout<<"aft "<<pts.size()<<std::endl;
	
	for(typename std::vector<Eigen::Matrix<Scalar, 2, 1> >::const_iterator it=++pts.begin(); ;) {
		boost::shared_ptr<Triangle> tr(new Triangle);
		tr->copy(*this);
		tr->p_[0] = pts.front();
		tr->p_[1] = *it;
		it++;
		if(it==pts.end()) break;
		tr->p_[2] = *it;
		
		if( ((*descr_)[tr->p_[1]]-(*descr_)[tr->p_[0]]).cross((*descr_)[tr->p_[2]]-(*descr_)[tr->p_[0]]).squaredNorm()<0.000001) continue;
		
		tr->reset();
		tr->compute(samples);
		res.push_back(tr);
	}
}

#if 0
template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::subsample(const boost::shared_ptr<Triangle> &_this, const typename S::Samples &samples, const TVector &at, const Scalar r2, std::vector<boost::shared_ptr<Triangle> > &res) const {
	//brute force (for the start)
	bool b[3], success;
	int n=0;
	for(int j=0; j<3; j++) {
		n+= (b[j] = ( (this->p3_[j]-at).squaredNorm()<=r2))?1:0;
		assert(this->at(p_[j]) == this->p3_[j]);
	}

	std::list<boost::shared_ptr<Triangle> > trs, trs_good;
	int indA=0, indB=0;
	
	assert(this->computed_);
	assert(_this.get()==this);
	
	if(n==0) {
		//check if sphere intersects only partially
		for(int j=0; j<3; j++) {
			Eigen::Matrix<Scalar, 2, 1> rv1,rv2;
			if(descr_->intersection_on_line(at, r2, p_[j], p_[ (j+1)%3 ], rv1,rv2)) {
				boost::shared_ptr<Triangle> tr(new Triangle);
				tr->copy(*this);
				tr->p_[j]       = rv1;
				tr->p_[(j+1)%3] = rv2;
				tr->p_[(j+2)%3] = descr_->intersection_on_line(at, r2, (rv1+rv2)/2, p_[ (j+2)%3 ], success);
				indA=j;
				indB=(j+1)%3;
				trs.push_back(tr);
			}
		}
		if(trs.size()) std::cout<<trs.size()<<std::endl;
		//assert(trs.size()<=1);
		if(trs.empty()) return;
	}
	else if(n==3) {
		res.push_back(_this);
		return;
	}
	else if(this->getArea()<0.005*0.005) //ignore as it is too small
		return;	//TODO: threshold
	else if(n==2) {
		boost::shared_ptr<Triangle> tr1(new Triangle), tr2(new Triangle);
		tr1->copy(*this);
		tr2->copy(*this);
		const int ind = !b[0]?0:(!b[1]?1:2);
		
		tr1->p_[ind] = descr_->intersection_on_line(at, r2, p_[ind], p_[ (ind+2)%3 ], success);
		//assert(success);
		tr2->p_[(ind+2)%3] = descr_->intersection_on_line(at, r2, p_[ind], p_[ (ind+1)%3 ], success);
		//assert(success);
		tr2->p_[ind] = tr1->p_[ind];
		
		trs.push_back(tr2);
		indA=ind;
		indB=(ind+2)%3;
		
		tr1->reset();
		tr1->compute(samples);
		res.push_back(tr1);
	}
	else if(n==1) {
		boost::shared_ptr<Triangle> tr(new Triangle);
		tr->copy(*this);
		const int ind = b[0]?0:(b[1]?1:2);
		
		tr->p_[(ind+1)%3] = descr_->intersection_on_line(at, r2, tr->p_[ind], tr->p_[ (ind+1)%3 ], success);
		//assert(success);
		tr->p_[(ind+2)%3] = descr_->intersection_on_line(at, r2, tr->p_[ind], tr->p_[ (ind+2)%3 ], success);
		//assert(success);
		
		trs.push_back(tr);
		indA=(ind+1)%3;
		indB=(ind+2)%3;
	}
	
	//subsample further
	int num=0;
	while(num<4 && trs.size()>0) {
		++num;
		const Scalar er = ( ((this->at( (trs.back()->p_[indA]+trs.back()->p_[indB])/2 ) - at).squaredNorm()/r2)-1 );
		if( er > 0.1) { //TODO: set threshold
			boost::shared_ptr<Triangle> a=trs.back(), b(new Triangle);
			b->copy(*trs.back());
			a->p_[indA]=b->p_[indB]=descr_->intersection_on_line(at, r2, a->p_[indA], a->p_[indB], success);
			a->reset();
			if(success) {
				trs.push_back(b);
			} else {
				trs_good.push_back(trs.back());
				trs.pop_back();
			}
		}
		else {
			trs_good.push_back(trs.back());
			trs.pop_back();
		}
	}
	
	for(typename std::list<boost::shared_ptr<Triangle> >::iterator it=trs_good.begin(); it!=trs_good.end(); it++) {
		(*it)->reset();
		(*it)->compute(samples);
		res.push_back(*it);
	}
	for(typename std::list<boost::shared_ptr<Triangle> >::iterator it=trs.begin(); it!=trs.end(); it++) {
		(*it)->reset();
		(*it)->compute(samples);
		res.push_back(*it);
	}
}
#endif

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::subsample(const TVector &at, const Scalar r2, std::vector<boost::shared_ptr<Triangle> > &res, const std::vector<boost::shared_ptr<Triangle> > &map) const {
	res.clear();
	for(size_t i=0; i<map.size(); i++)
		map[i]->subsample(map[i], samples_, at, r2, res);
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::__evalKeypoints__setInput(PTSurfaceList surfs, const Real min_area, const Real area) {
	kp_min_area_ = min_area;
	kp_area_ = area;
	
	input_=surfs;
	triangulated_input_.clear();

	//generate keypoints (e.g. reduce number of points by area)
	keypoints_.clear();
	all_keypoints_.clear();
	generateKeypoints(all_keypoints_);
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::setInput(PTSurfaceList surfs) {
	input_=surfs;
	triangulated_input_.clear();

	//generate keypoints (e.g. reduce number of points by area)
	keypoints_.clear();
	all_keypoints_.clear();
	generateKeypoints(all_keypoints_);

	//remove keypoints which are not within FoV
	//and filter keypoints (minimum distance to each other)
	const double R = (radii_[0]);	
	for(size_t i=0; i<all_keypoints_.size(); i++) {
		if(all_keypoints_[i].squaredNorm()>4.*4.) continue; //TODO: set threshold
		
		bool ok=true;
		for(size_t j=0; j<i; j++) {
		  if( (all_keypoints_[i]-all_keypoints_[j]).squaredNorm()<0.6f /*HACK*/ /8 ) {//TODO: set threshold
			ok=false;
			break;
		  }
		}
		
		for(size_t j=0; ok && j<fov_.size(); j++) {
		  if( fov_[j].head<3>().dot(all_keypoints_[i].template cast<float>())-fov_[j](3)>-R ) {
			ok = false;
			break;
		  }
		}
		
		if(ok) keypoints_.push_back(all_keypoints_[i]);
	}
  
	//std::cout<<"got "<<keypoints_.size()<<" keypoints"<<std::endl;
	
	#pragma omp parallel for num_threads( NUM_THREADS )
	for(size_t indx=0; indx<input_->size(); indx++) {
		std::vector<boost::shared_ptr<Triangle> > trs = (*input_)[indx].template generateTriangles<Triangle>();
		for(size_t l=0; l<trs.size(); l++)
			trs[l]->compute(samples_);
		
      	#pragma omp critical
      	triangulated_input_.insert(triangulated_input_.end(), trs.begin(), trs.end());
      	/*
		  TPPLPartition pp;
		  std::list<TPPLPoly> polys, result;

		  //fill polys
		  for(size_t i=0; i<(*input_)[indx].segments_.size(); i++) {
			TPPLPoly poly;
			poly.Init((*input_)[indx].segments_[i].size());
			poly.SetHole(i!=0);

			for(size_t j=0; j<(*input_)[indx].segments_[i].size(); j++) {
			  poly[j].x = (*input_)[indx].segments_[i][j](0);
			  poly[j].y = (*input_)[indx].segments_[i][j](1);
			}
			
			poly.SetOrientation(i!=0?TPPL_CW:TPPL_CCW);
			polys.push_back(poly);
			//break;
		  }

		  if(polys.size()<1) continue;
		  pp.Triangulate_EC(&polys, &result);

		  Eigen::Vector3f v1,v2,v3;
		  for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
			assert(it->GetNumPoints()==3);

			boost::shared_ptr<Triangle> tr(new Triangle);
			tr->set(&(*input_)[indx].model_, it->GetPoint(0), it->GetPoint(1), it->GetPoint(2));
			tr->compute(samples_);
			
      			#pragma omp critical
			triangulated_input_.push_back(tr);
		  }*/
	}
	
	//std::cout<<"triangles: "<<triangulated_input_.size()<<std::endl;
}


template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::dbg_writeVolumeImage(const std::string &fn) {
	std::ofstream of(fn.c_str());
	of<<"[";
	for(size_t i=0; i<sr_.vals_.size(); i++) {
		if(i) of<<",";
		of<<"[";
		for(size_t j=0; j<sr_.vals_[i].size(); j++) {
			if(j) of<<",";
			of<<sr_.vals_[i][j];
		}
		of<<"]";
	}
	of<<"]";
	of.close();
}
