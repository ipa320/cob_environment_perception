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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace cob_3d_mapping_features
{
  /*! @brief feature generator for surfaces based on fourier transformation (rotation/translation invariant) */
  template<typename TSurface, typename Scalar=float, typename TAffine=Eigen::Affine3f, typename TVector=Eigen::Vector3f> class InvariantSurfaceFeature
  {
  public:
    typedef std::vector<TSurface> TSurfaceList;
    typedef boost::shared_ptr<TSurfaceList> PTSurfaceList;
    typedef Eigen::Matrix<Scalar, num_radius_*num_angle_*num_angle_, 1> Feature;

    struct ResultVector {
      TVector v;
      std::vector<Feature> ft;
    };
    typedef std::vector<ResultVector> ResultVectorList;
    typedef boost::shared_ptr<ResultVectorList> PResultVectorList;

    typedef enum {
      INVARAINCE_X=1, INVARAINCE_Y=2, INVARAINCE_Z=4,
      INVARAINCE_INCLINATION=8, INVARAINCE_AZIMUTH=16,
      INVARAINCE_ALL=0x1f
    } EINVARAINCE;

    /*! constructor */
    InvariantSurfaceFeature() :
      invariance_(INVARAINCE_ALL), num_radius_(4), num_angle_(4)
    {
      //TODO: default radi
    }

    /*! destructor */
    ~InvariantSurfaceFeature();

    void setInput(PTSurfaceList surfs);
    void compute();

    const TAffine &getTransformation() const {return transform_;}
    void setTransformation(const TAffine &t) {transform_=t;}

    const EINVARAINCE &getInvarianceSettings() const {return invariance_;}
    void setInvarianceSettings(const EINVARAINCE &t) {invariance_=t;}

    const std::vector<float> &getRadii() const {return radii_;}
    void addRadii(const float r) {	//insert sorted!
		std::vector<float>::iterator it = radii_.begin();
		while(it!=radii_.end() && *it>r) ++it;
		radii_.insert(it, r);
	}

    const int getNumRadius() const {return num_radius_;}
    void setNumRadius(const int t) {num_radius_=t;}
    const int getNumAngle() const {return num_angle_;}
    void setNumAngle(const int t) {num_angle_=t;}

  protected:
    struct Triangle {
		Eigen::Matrix<Scalar, 2, 1> p_[3];
		Eigen::Matrix<Scalar, 3, 1> p3_[3];
		Feature f_;
		typename TSurface::Model *model_;
		
		inline static set(Eigen::Matrix<Scalar, 2, 1> &p, const TPPLPoint &tp) {
			p(0) = tp.x;
			p(1) = tp.y;
		}
		
		inline static int index(const int a1, const int a2, const int r) {
			return a1 + a2*num_angle_ + r*num_angle_*num_angle_;
		}
		
		void compute();
	};
	
    struct VectorWithParams {
      TVector v_;
      typename TSurface::TParam::VectorU *param_;
    };
    
    PTSurfaceList input_;
    std::vector<Triangle> triangulated_input_;
    
    TAffine transform_;
    EINVARAINCE invariance_;
    std::vector<float> radii_;	//descending sorted radius list (reuse of subsampled map)
    PResultVectorList result_;
    int num_radius_, num_angle_;

    void generateKeypoints(std::vector<TVector> &keypoints);
    void subsample(const TVector &at, const Scalar r2, std::vector<Triangle> &res);
    void kernel();
  };
}
