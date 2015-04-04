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

#include <complex>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <libpolypartition/polypartition.h>
#include <pcl/PolygonMesh.h>		//only used for debugging/test functions (perhaps put outside?)

#include "ShapeSPH/Util/Signature.h"
#include "ShapeSPH/Util/SphereSampler.h"
#include "ShapeSPH/Util/lineqn.h"
#include "ShapeSPH/Util/SphericalPolynomials.h"
#include "invariant_surface_feature/triangle.hpp"

namespace cob_3d_features
{
  template<typename PSurface>
  class PolynomialSurfaceHelper : public PSurface
  {
	  public:
	  using PSurface::Model;
	  using PSurface::segments_;
	  using PSurface::model_;
	  //typedef typename PSurface::Model Model;
	  
	  PolynomialSurfaceHelper(const PSurface &s) : PSurface(s) {}
	  
	  template<typename TVector, typename Real>
	  void generateKeypoints(std::vector<TVector> &keypoints, const Real kp_min_area_, const Real kp_area_);
	  
	  template<typename Scalar>
	  bool intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, Eigen::Matrix<Scalar, 2, 1> &res1, Eigen::Matrix<Scalar, 2, 1> &res2, bool &res_b1, bool &res_b2) const;
		  
	  template<typename Scalar>
      Eigen::Matrix<Scalar, 2, 1> intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, bool &success) const;
      
      inline float operator()(const float x, const float y) const {return model_.model(x,y);}
      template<typename S>
      inline Eigen::Matrix<S, 3, 1> operator[](const Eigen::Matrix<S,2,1> &p) const {
		  Eigen::Matrix<S, 3, 1> r;
		  r(2) = (*this)(r(0)=p(0),r(1)=p(1));
		  return r; 
	  }
	  
	  static PolynomialSurfaceHelper parse(const cob_3d_mapping_msgs::Shape &shape) {
			PSurface p;
			assert((int)shape.params.size() <= (int)p.model_.p.size());
			for(size_t j=0; j<shape.params.size(); j++)
				p.model_.p(j) = shape.params[j];
				
			p.segments_.resize(shape.points.size());
			for(size_t j=0; j<shape.points.size(); j++) {
				//std::cout<<j<<" "<<(j!=0)<<" "<<(bool)shape.holes[j]<<std::endl;
				//assert( (j!=0)==(bool)shape.holes[j] );
				
				pcl::PointCloud<pcl::PointXYZ> pc;
				pcl::fromROSMsg(shape.points[j], pc);

				for(size_t k=0; k<pc.size(); k++)
					p.segments_[j].push_back(pc[k].getVector3fMap());
			}
			return p;
		}
      
	  template<typename Triangle>
	  std::vector<boost::shared_ptr<Triangle> > generateTriangles() {
		  std::vector<boost::shared_ptr<Triangle> > ret;
		  
		  TPPLPartition pp;
		  std::list<TPPLPoly> polys, result;

		  //fill polys
		  for(size_t i=0; i<segments_.size(); i++) {
			TPPLPoly poly;
			poly.Init(segments_[i].size());
			poly.SetHole(i!=0);

			for(size_t j=0; j<segments_[i].size(); j++) {
			  poly[j].x = segments_[i][j](0);
			  poly[j].y = segments_[i][j](1);
			}
			
			poly.SetOrientation(i!=0?TPPL_CW:TPPL_CCW);
			polys.push_back(poly);
			//break;
		  }

		  if(polys.size()<1) return ret;
		  pp.Triangulate_EC(&polys, &result);

		  Eigen::Vector3f v1,v2,v3;
		  for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
			assert(it->GetNumPoints()==3);

			boost::shared_ptr<Triangle> tr(new Triangle);
			tr->set(this, it->GetPoint(0), it->GetPoint(1), it->GetPoint(2));			
			ret.push_back(tr);
		  }
		  
		  return ret;
	  }
  };
  
  template<typename Shape>
  class TriangleSurfaceHelper : public Shape
  {
	  public:
	  using Shape::contours_;
	  
	  struct Model {};
	  
	  template<typename Triangle>
	  std::vector<boost::shared_ptr<Triangle> > generateTriangles() {
		  std::vector<boost::shared_ptr<Triangle> > ret;
		  
		  TPPLPartition pp;
		  std::list<TPPLPoly> polys, result;

		  //fill polys
		  for(size_t i=0; i<contours_.size(); i++) {
			TPPLPoly poly;
			poly.Init(contours_[i].size());
			poly.SetHole(i!=0);

			for(size_t j=0; j<contours_[i].size(); j++) {
			  poly[j].x = contours_[i][j](0);
			  poly[j].y = contours_[i][j](1);
			}
			
			poly.SetOrientation(i!=0?TPPL_CW:TPPL_CCW);
			polys.push_back(poly);
			//break;
		  }

		  if(polys.size()<1) return ret;
		  pp.Triangulate_EC(&polys, &result);

		  Eigen::Vector3f v1,v2,v3;
		  for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
			assert(it->GetNumPoints()==3);

			boost::shared_ptr<Triangle> tr(new Triangle);
			tr->set(this, it->GetPoint(0), it->GetPoint(1), it->GetPoint(2));			
			ret.push_back(tr);
		  }
		  
		  return ret;
	  }
	  
	  static TriangleSurfaceHelper parse(const cob_3d_mapping_msgs::Shape &shape) {
		  TriangleSurfaceHelper r;
		  fromROSMsg(shape, r);
		  return r;
	  }
	  
	  template<typename TVector, typename Real>
	  void generateKeypoints(std::vector<TVector> &keypoints, const Real kp_min_area_, const Real kp_area_);
	  
	  template<typename Scalar>
	  bool intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, Eigen::Matrix<Scalar, 2, 1> &res1, Eigen::Matrix<Scalar, 2, 1> &res2, bool &res_b1, bool &res_b2) const;
		  
	  template<typename Scalar>
      Eigen::Matrix<Scalar, 2, 1> intersection_on_line(const Eigen::Matrix<Scalar, 3, 1> &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, bool &success) const;
      
      template<typename S>
      inline Eigen::Matrix<S, 3, 1> operator[](const Eigen::Matrix<S,2,1> &p) const {
		  return this->Shape::operator[](p.template cast<float>()).template cast<S>();
	  }
  };
  
  /*! @brief feature generator for surfaces based on fourier transformation (rotation/translation invariant) */
  template<typename _TSurface, typename Scalar=double, typename Real=float, typename _TAffine=Eigen::Affine3f>
  class InvariantSurfaceFeature
  {
	  typedef Sampler<Real, Scalar> S;
	  
	  const int num_radius_, num_angle_;
	  S sr_;
	  typename S::Samples samples_, samples_origin_;
	  
	  Real kp_min_area_, kp_area_;
  public:
  
    typedef Eigen::Matrix<Scalar, 3, 1> TVector;
    typedef _TAffine TAffine;
    typedef _TSurface TSurface;
    
	struct Feature {
		typedef std::vector<Signature< Real > > FeatureVector;
		FeatureVector f_;
		TVector pt_;
		TSurface *descr_;
		Real area_;
		
		Feature(const TVector &pt, TSurface *descr=NULL) :
			pt_(pt), descr_(descr), area_(0)
		{}
		
		//simple manhatten distance (for testing)
		Real distance(const Feature &b) const {
			assert(f_.size()==b.f_.size());
			Real r=0;
			for(size_t i=0; i<f_.size(); i++) {
				assert(f_[i].size()==b.f_[i].size());
				for(size_t j=0; j<(size_t)f_[i].size(); j++)
					r += std::abs(f_[i][j]-b.f_[i][j]);
			}
			return r;
		}
		
		void normalize(const std::vector<float> &radii) {
			return;
			
			//std::cout<<"area: "<<area_<<std::endl;
			//return;
			/*for(size_t i=0; i<f_.size(); i++)
				for(size_t j=0; j<(size_t)f_[i].size(); j++)
					f_[i][j]/=std::sqrt(radii[i]);
			return;
			const int N=16;*/
			for(size_t i=0; i<f_.size(); i++)
				for(size_t j=0; j<(size_t)f_[i].size(); j++)
					//f_[i][j]*=(j%N==0)?0.5:1.;
					f_[i][j]/=area_;// *std::sqrt((Real)( j/N+1) );
					//f_[i][j]/=f_[i][(j/N)*N]*(j/N+1);
					//f_[i][j]/=(j/N+1);
		}
		
		void toVector(std::vector<float> &res) const {
			res.clear();
			for(size_t i=0; i<f_.size(); i++)
				for(size_t j=0; j<(size_t)f_[i].size(); j++)
					res.push_back(f_[i][j]);
		}
	};

    typedef std::vector<TSurface> TSurfaceList;
    typedef boost::shared_ptr<TSurfaceList> PTSurfaceList;
    
    typedef std::vector<Feature> Result;
    typedef boost::shared_ptr<Result> PResult;
    typedef boost::shared_ptr<const Result> PResultConst;

    typedef enum {
      INVARAINCE_X=1, INVARAINCE_Y=2, INVARAINCE_Z=4,
      INVARAINCE_INCLINATION=8, INVARAINCE_AZIMUTH=16,
      INVARAINCE_ALL=0x1f
    } EINVARAINCE;

    /*! constructor */
    InvariantSurfaceFeature(const int num_radius, const int num_angle) :
	  num_radius_(num_radius), num_angle_(num_angle),
	  sr_((Real)num_radius, num_radius, num_angle),
      kp_min_area_(0.02*0.02), kp_area_(0.05*0.05),
      invariance_(INVARAINCE_ALL)
    {
		sr_.getSamples(samples_origin_);
		samples_ = samples_origin_;
		//TODO: default radii
    }

    /*! destructor */
    virtual ~InvariantSurfaceFeature() {}

	//resets the input (list of surfaces with holes) and computes initial fft
    void setInput(PTSurfaceList surfs);
    void compute();
    
    void __evalKeypoints__setInput(PTSurfaceList surfs, const Real min_area, const Real area);

    PResultConst getResult() const {return result_;}

    const TAffine &getTransformation() const {return transform_;}
    void setTransformation(const TAffine &t) {transform_=t;}

    const EINVARAINCE &getInvarianceSettings() const {return invariance_;}
    void setInvarianceSettings(const EINVARAINCE &t) {invariance_=t;}

    const std::vector<float> &getRadii() const {return radii_;}
    void addRadius(float r) {	//insert sorted!
		r*=r;
		std::vector<float>::iterator it = radii_.begin();
		while(it!=radii_.end() && *it>r) ++it;
		radii_.insert(it, r);
		
		if(r==radii_.front()) {
			sr_.setMaxRadius(num_radius_/r);
			sr_.getSamples(samples_origin_);
			samples_ = samples_origin_;
		}
	}
	
	void setFoV(const std::vector<Eigen::Vector4f> &fov) {fov_=fov;}
	const std::vector<Eigen::Vector4f> &getFoV() const {return fov_;}

	void rotateSamples(const Eigen::Matrix<Scalar,3,3> &R) {
		for(size_t i=0; i<samples_.size(); i++)
			for(size_t j=0; j<samples_[i].size(); j++)
				samples_[i][j] = R*samples_origin_[i][j];
	}

    /* UNIT TESTS: available in invariant_surface_feature_unit_tests.hpp */
    bool test_singleTriangle(const int num) const;
	pcl::PolygonMesh::Ptr test_subsampling_of_Map(const int num, const Scalar r2);
	void test_addOffset(const Scalar off_x, const Scalar off_y, const Scalar off_z);
	void test_rotate(const Scalar angle);
	
	/* DEBUG functions */
	void dbg_writeVolumeImage(const std::string &fn);
    void dbg_mesh_of_subsamp(const TVector &at, const Scalar radius, std::vector<TVector> &pts, std::vector<int> &inds) const;
    void dbg_keypoints(std::vector<TVector> &keypoints) const {generateKeypoints(keypoints);}
	pcl::PolygonMesh::Ptr dbg_Mesh_of_Map() const {return dbg_triangles2mesh(triangulated_input_);}

	const std::vector<TVector> &getKeypoints() const {return keypoints_;}
	const std::vector<TVector> &getAllKeypoints() const {return all_keypoints_;}
  protected:

	template<class Model>
	static inline Eigen::Matrix<Scalar, 3, 1> at(const Eigen::Matrix<Scalar, 2, 1> &p, const Model &model) {
		Eigen::Matrix<Scalar, 3, 1> v;
		v(0) = p(0); v(1) = p(1);
		v(2) = model.model(p(0),p(1));
		return v;
	}
	template<class Model>
	static inline Eigen::Matrix<Scalar, 3, 1> at(const Eigen::Matrix<Real, 2, 1> &p, const Model &model) {
		Eigen::Matrix<Scalar, 3, 1> v;
		v(0) = p(0); v(1) = p(1);
		v(2) = model.model(p(0),p(1));
		return v;
	}
		
    struct Triangle : public cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, typename S::Samples, typename S::Values> {
	private:
		typedef cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, typename S::Samples, typename S::Values> Base;
		Eigen::Matrix<Scalar, 2, 1> p_[3];
		TSurface *descr_;
		
	public:
		typedef boost::shared_ptr<Triangle> Ptr;
	
		inline static void set(Eigen::Matrix<Scalar, 2, 1> &p, const TPPLPoint &tp) {
			p(0) = tp.x;
			p(1) = tp.y;
		}
		
		void copy(const Triangle &o) {	//keeps f_ uninitialized!
			this->Base::copy(o);
			descr_ = o.descr_;
			for(int i=0; i<3; i++)
				p_[i] = o.p_[i];
			reset();
		}
		
		void reset() {
			this->computed_ = false;	//reset
			this->cr = -1;
		}
		
		void set(TSurface *descr, const TPPLPoint &p1, const TPPLPoint &p2, const TPPLPoint &p3) {
			descr_ = descr;
			set(p_[0], p1);
			set(p_[1], p2);
			set(p_[2], p3);
			reset();
		}
		
		void set(TSurface *descr, const Eigen::Matrix<Scalar, 2, 1> &p1, const Eigen::Matrix<Scalar, 2, 1> &p2, const Eigen::Matrix<Scalar, 2, 1> &p3) {
			descr_ = descr;
			p_[0] = p1;
			p_[1] = p2;
			p_[2] = p3;
			reset();
		}
		
		void compute(const typename S::Samples &samples);
		void subsample(const boost::shared_ptr<Triangle> &_this, const typename S::Samples &samples, const TVector &at, const Scalar r2, std::vector<boost::shared_ptr<Triangle> > &res) const;
		std::complex<Scalar> kernel(const Scalar m, const Scalar n, const Scalar p) const;

		void print() const;
    private:
		//Eigen::Matrix<Scalar, 2, 1> intersection_on_line(const TVector &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, bool &success) const;
		//bool intersection_on_line(const TVector &at, const Scalar r2, const Eigen::Matrix<Scalar, 2, 1> &a, const Eigen::Matrix<Scalar, 2, 1> &b, Eigen::Matrix<Scalar, 2, 1> &res1, Eigen::Matrix<Scalar, 2, 1> &res2, bool &res_b1, bool &res_b2) const;
		//std::complex<Scalar> sub_kernel(const Scalar m, const Scalar n, const Scalar p, const Tri2D &tri, const int depth=0) const;
		std::complex<Scalar> kernel_lin(const Scalar m, const Scalar n, const Scalar p, const Scalar x0, const Scalar y0, const Scalar y1, const Scalar d1, const Scalar d2) const;
		//std::complex<Scalar> kernel_lin_tri(const Scalar m, const Scalar n, const Scalar p, const Tri2D &tri) const;

		inline Eigen::Matrix<Scalar, 3, 1> at(const Scalar x, const Scalar y) const {
			return at(Eigen::Matrix<Scalar, 2, 1>(x,y));
		}

		inline Eigen::Matrix<Scalar, 3, 1> at(const Eigen::Matrix<Scalar, 2, 1> &p) const {
			return (*descr_)[p];
		}

		Scalar area() const {
			//TODO: at the moment kind of "approximation"
			const Eigen::Matrix<Scalar, 2, 1> mid2 = ((p_[0])+(p_[1])+(p_[2]))/3;
			Eigen::Matrix<Scalar, 3, 1> mid23, mid3, v1,v2,v3;

			v1 = (*descr_)[p_[0]];
			v2 = (*descr_)[p_[1]];
			v3 = (*descr_)[p_[2]];

			mid23 = (*descr_)[mid2];
			mid3 = mid23;
			mid3(2) =  (v1(2)+v2(2)+v3(2))/3;

//std::cout<<"area "<<(mid3 - mid23).squaredNorm()<<" "<<mid23(2)<<" "<<mid3(2)<<std::endl;
			return (mid3 - mid23).norm()*(v1-v2).cross(v3-v2).norm();
		}
	};
	
    struct VectorWithParams {
      TVector v_;
      typename TSurface::TParam::VectorU *param_;
    };
    
    PTSurfaceList input_;
    std::vector<boost::shared_ptr<Triangle> > triangulated_input_;
	std::vector<TVector> keypoints_, all_keypoints_;
    
    TAffine transform_;
    EINVARAINCE invariance_;
    std::vector<float> radii_;	//descending sorted radius list (reuse of subsampled map)
    PResult result_;
	std::vector<Eigen::Vector4f> fov_; /// [normalized normal vector of plane, offset]

    void generateKeypoints(std::vector<TVector> &keypoints) const;
    void subsample(const TVector &at, const Scalar r2, std::vector<boost::shared_ptr<Triangle> > &res, const std::vector<boost::shared_ptr<Triangle> > &map) const;
    
    //simple manhatten distance (for testing)
    Real distance(const Result &a, const Result &b) {
		assert(a.size()==b.size());
		Real r=0;
		for(size_t i=0; i<a.size(); i++) {
			assert(a[i].size()==b[i].size());
			for(size_t j=0; j<a[i].size(); j++)
				r += std::abs(a[i][j]-b[i][j]);
		}
		return r;
	}
	
	/* DEBUG functions */
	pcl::PolygonMesh::Ptr dbg_triangles2mesh(const std::vector<boost::shared_ptr<Triangle> > &res) const;
  };
}
