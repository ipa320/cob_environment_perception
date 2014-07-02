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

  for(size_t i=0; i<keypoints_.size(); i++)  {
	result_->push_back( Feature(keypoints_[i]) );
	result_->back().f_.resize(radii_.size());
  }
	
	static int dbg_stage=0, dbg_num=0;
	++dbg_stage;
	
  #pragma omp parallel for num_threads( 4 )
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
		for(int i=0; i<3; i++) {
			this->p3_[i](0) = p_[i](0);
			this->p3_[i](1) = p_[i](1);
			this->p3_[i](2) = model_->model(p_[i](0), p_[i](1));
		}
	
		this->d1 = this->p3_[2]-this->p3_[1];
		this->d2 = this->p3_[1]-this->p3_[0];
		this->cr = this->d1.cross(this->d2).norm();
	}
	
	//check if further sub-sampling is necessary?
	if(area<TSurface::DEGREE>()>0.0001/*||rand()%13==0||depth<4*/) {
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
			tris[i].model_ = model_;
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

  for(size_t i=0; i<input_->size(); i++) {                         //surfaces
    for(size_t j=0; j<(*input_)[i].segments_.size(); j++) {        //outline/holes
    
      //std::cout<<"size bef "<<(*input_)[i].segments_[j].size()<<std::endl;
    
	  typedef std::list<Eigen::Matrix<Real, 2, 1> > LIST;
	  LIST pts;
	  float area = 0.f, min_area;
	  const Real FACTOR = 0.05f;
	  const Real FACTOR_MIN = 0.0025f;
	  
      for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++)   //points
      {
		  Eigen::Matrix<Real, 2, 2> M;
		  M.col(0) = (*input_)[i].segments_[j][k ].head(2);
		  M.col(1) = (*input_)[i].segments_[j][(k+1)%(*input_)[i].segments_[j].size()].head(2);
		  area -= M.determinant()/2;
		  
		  pts.push_back((*input_)[i].segments_[j][k].head(2));
	  }
	  //assert(area>=0);
	  area = std::abs(area);
	  
	  min_area = std::max(area*FACTOR_MIN, (Real)(0.02*0.02)); //TODO: set threshold [cm^2]
	  area*=FACTOR;
	  area = std::max(area, (Real)(0.075*0.075));	//TODO: set threshold [cm^2]
	  
	  while(pts.size()>=3) {
		  bool rem = false;	//if nothing was removed, calc. remaining area
		  float mi = std::numeric_limits<float>::max();	//minimum weight
		  typename LIST::iterator mit = pts.end();	//iterator to minim weighted pt.
		  float remaining_area = 0.f;	//if remaining area is already smaller than threshold --> break
		  
		  /*static int nnn=0;
		  std::ofstream of( ("/tmp/poly"+boost::lexical_cast<std::string>(nnn++)+".svg").c_str() );
		  of<<"<svg height=\"500\" width=\"500\"><polygon points=\"";
		  for(typename LIST::iterator it=pts.begin(); it!=pts.end(); it++) {
			  of<<(*it)(0)*300+200<<","<<(*it)(1)*300+200<<" ";
		  }
		  of<<"\" /></svg>";
		  of.close();*/
		  
		  for(typename LIST::iterator it=pts.begin(); it!=pts.end(); it++) {
			  typename LIST::iterator n1 = it, n2=it;
			  n1++; n2++; if(n1==pts.end()) n1=n2=pts.begin();
			  n2++; if(n2==pts.end()) n2=pts.begin();
			  
			  Eigen::Matrix<Real, 2, 2> M;
			  M.col(0) = (*it-*n1);
			  M.col(1) = (*n2-*n1);
			  const float w = std::abs(M.determinant()/2);
			  //std::cout<<"w "<<w<<std::endl;
			  
			  if(!rem) {
				  M.col(0) = *it;
				  M.col(1) = *n1;
				  remaining_area-=M.determinant()/2;
			  }
			  
			  /*if(w>=area) {	//fulfills already our criteria
			  //std::cout<<"rem"<<std::endl;
				  keypoints.push_back(this->at(*n1, (*input_)[i].model_));
				  pts.erase(n1);
				  it--;
				  rem = true;
			  } else*/ if(w<mi) {
				mit = n1;
				mi = w;
			}
		  }
		  
		  if(mi>area) {	//fulfills already our criteria
			  //std::cout<<"add keypoints "<<pts.size()<<std::endl;
			  for(typename LIST::const_iterator pit=pts.begin(); pit!=pts.end(); pit++) keypoints.push_back(this->at(*pit, (*input_)[i].model_));
			  break;
		  }
		  
		  //if(rem) continue;
			  //std::cout<<"s "<<pts.size()<<" "<<(mit_n)<<std::endl;
		  //if( (!rem && remaining_area<area) || pts.size()<3) break;
		  assert(mit!=pts.end());
		  
		  
		  if(mi<min_area) {
			  //TODO: improve
			  for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++)   //points
				if((*input_)[i].segments_[j][k].head(2)==*mit) {
					(*input_)[i].segments_[j].erase((*input_)[i].segments_[j].begin()+k);
					//std::cout<<"reduced by one point"<<std::endl;
					break;
				}
		  }
		  pts.erase(mit);	//remove point with minimum weight --> increases weight of surroundings
	  }
	  
      //std::cout<<"size aft "<<(*input_)[i].segments_[j].size()<<std::endl;
	}
  }

  //remove keypoints which are not within FoV
  const double R = (radii_[0]);
  for(size_t i=0; i<keypoints.size(); i++) {
	  for(size_t j=0; j<fov_.size(); j++) {
		  if( fov_[j].head<3>().dot(keypoints[i].template cast<float>())-fov_[j](3)>-R ) {
			keypoints.erase(keypoints.begin()+i);
			--i;
			break;
		  }
	  }
  }
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
Eigen::Matrix<Scalar, 2, 1>
cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::intersection_on_line(const TVector &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, bool &success) const {

//assert((this->at(a)-at).squaredNorm()>r2 || (this->at(b)-at).squaredNorm()>r2);

	Eigen::PolynomialSolver<typename TSurface::Model::VectorU1D::Scalar, 2*TSurface::DEGREE> solver;	
	typename TSurface::Model::VectorU1D p = model_->transformation_1D( (b-a).template cast<typename TSurface::Model::VectorU1D::Scalar>(), a.template cast<typename TSurface::Model::VectorU1D::Scalar>(), at.template cast<typename TSurface::Model::VectorU1D::Scalar>());
	p(0) -= r2;
	solver.compute(p);
	
	std::vector<typename TSurface::Model::VectorU1D::Scalar> r;
	solver.realRoots(r);
	assert(r.size()>0);
	
	const typename TSurface::Model::VectorU1D::Scalar mid = 0.5f;
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

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
bool
cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::Triangle::intersection_on_line(const TVector &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, Eigen::Matrix<Scalar, 2, 1> &res1, Eigen::Matrix<Scalar, 2, 1> &res2, bool &res_b1, bool &res_b2) const {
	Eigen::PolynomialSolver<typename TSurface::Model::VectorU1D::Scalar, 2*TSurface::DEGREE> solver;	
	typename TSurface::Model::VectorU1D p = model_->transformation_1D( (b-a).template cast<typename TSurface::Model::VectorU1D::Scalar>(), a.template cast<typename TSurface::Model::VectorU1D::Scalar>(), at.template cast<typename TSurface::Model::VectorU1D::Scalar>());
	p(0) -= r2;
	solver.compute(p);
	
	std::vector<typename TSurface::Model::VectorU1D::Scalar> r;
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
	
	std::vector<Eigen::Matrix<Scalar, 2, 1> > pts;
	std::vector<bool> pts_ins;
	
	int n=-1;
	for(int j=0; j<3; j++) {
		bool rb1, rb2;
		Eigen::Matrix<Scalar, 2, 1> rv1,rv2;
		if(intersection_on_line(at, r2, p_[j], p_[ (j+1)%3 ], rv1,rv2, rb1, rb2)) {
			pts.push_back(rv1);
			pts.push_back(rv2);
			pts_ins.push_back(rb1);
			pts_ins.push_back(rb2);
			n=j;
		}
	}
	
	size_t p=0;
	
	if(pts.size()<2) return;
	else if(pts.size()==2) {
		bool success;
		pts.push_back( intersection_on_line(at, r2, (pts.front()+pts.back())/2, p_[ (n+2)%3 ], success) );
		pts_ins.push_back(false);
		assert(success);
		++p;
	}
	
	//subsample further
	int num=0;
	//std::cout<<"bef "<<pts.size()<<std::endl;
	while(num<8 && p<pts.size()&&0) {
		if(pts_ins[p] || pts_ins[(p+1)%pts.size()]) {
			++p;
			continue;
		}
		const Scalar er = std::abs( ((this->at( (pts[p]+pts[(p+1)%pts.size()])/2 ) - at).squaredNorm()/r2)-1 );
	//std::cout<<"er "<<er<<std::endl;
		if( er > 0.02) { //TODO: set threshold
			bool success;
			Eigen::Matrix<Scalar, 2, 1> s = at.head(2), m = (pts[p]+pts[(p+1)%pts.size()])/2;
			Eigen::Matrix<Scalar, 2, 1> v=intersection_on_line(at, r2, (m-s)*2+s, s, success);
			if(success) {
				++num;
				pts.insert(pts.begin()+(p+1), v);
				pts_ins.insert(pts_ins.begin()+(p+1), false);
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
		
		Eigen::Matrix<Scalar, 3, 1> a,b;
		a.head(2) = tr->p_[1]-tr->p_[0];
		b.head(2) = tr->p_[2]-tr->p_[0];
		a(2)=b(2)=0;
		if( a.cross(b).squaredNorm()<0.000001) continue;
		
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
			if(intersection_on_line(at, r2, p_[j], p_[ (j+1)%3 ], rv1,rv2)) {
				boost::shared_ptr<Triangle> tr(new Triangle);
				tr->copy(*this);
				tr->p_[j]       = rv1;
				tr->p_[(j+1)%3] = rv2;
				tr->p_[(j+2)%3] = intersection_on_line(at, r2, (rv1+rv2)/2, p_[ (j+2)%3 ], success);
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
		
		tr1->p_[ind] = intersection_on_line(at, r2, p_[ind], p_[ (ind+2)%3 ], success);
		//assert(success);
		tr2->p_[(ind+2)%3] = intersection_on_line(at, r2, p_[ind], p_[ (ind+1)%3 ], success);
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
		
		tr->p_[(ind+1)%3] = intersection_on_line(at, r2, tr->p_[ind], tr->p_[ (ind+1)%3 ], success);
		//assert(success);
		tr->p_[(ind+2)%3] = intersection_on_line(at, r2, tr->p_[ind], tr->p_[ (ind+2)%3 ], success);
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
			a->p_[indA]=b->p_[indB]=intersection_on_line(at, r2, a->p_[indA], a->p_[indB], success);
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
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::setInput(PTSurfaceList surfs) {
	input_=surfs;
	triangulated_input_.clear();

	//generate keypoints (e.g. reduce number of points by area)
	keypoints_.clear();
	all_keypoints_.clear();
	generateKeypoints(all_keypoints_);
	
	//filter keypoints (minimum distance to each other)
	for(size_t i=0; i<all_keypoints_.size(); i++) {
		bool ok=true;
		for(size_t j=0; j<i; j++) {
		  if( (all_keypoints_[i]-all_keypoints_[j]).squaredNorm()<radii_[0]*radii_[0]/8 ) {//TODO: set threshold
			ok=false;
			break;
		  }
		}
		
		if(ok) keypoints_.push_back(all_keypoints_[i]);
	}
  
	std::cout<<"got "<<keypoints_.size()<<" keypoints"<<std::endl;
	
	for(size_t indx=0; indx<input_->size(); indx++) {
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
			break;
		  }

		  if(polys.size()<1) continue;
		  pp.Triangulate_EC(&polys, &result);

		  Eigen::Vector3f v1,v2,v3;
		  for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
			assert(it->GetNumPoints()==3);

			boost::shared_ptr<Triangle> tr(new Triangle);
			tr->set(&(*input_)[indx].model_, it->GetPoint(0), it->GetPoint(1), it->GetPoint(2));
			tr->compute(samples_);
			
			triangulated_input_.push_back(tr);
		  }
	}
	
	std::cout<<"triangles: "<<triangulated_input_.size()<<std::endl;
}

