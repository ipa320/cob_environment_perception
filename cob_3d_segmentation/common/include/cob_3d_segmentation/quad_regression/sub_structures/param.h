/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
/*
 * param.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef PARAM_H_
#define PARAM_H_

#define FAK( x ) ((x+2)*(x+1)/2)
#define ROUND_UP_POW2( x ) (x<=2?2:(x<=4?4:(x<=8?8:(x<=16?16:(x<=32?32:(x<=64?64:(x<128?128:x)))))))

/**
 * leaf of quad-tree
 * contains data for regression of 6 parameters
 */
template<int Degree>
struct Param {
#ifdef MIN_EIGEN_VECTOR
  typedef Eigen::Matrix<float, FAK(Degree), 1>    VectorU;
  typedef Eigen::Matrix<float, FAK(Degree), FAK(Degree)>    MatrixU;
#else
  typedef Eigen::Matrix<float, (ROUND_UP_POW2( FAK(Degree) )), 1>    VectorU;
  typedef Eigen::Matrix<float, (ROUND_UP_POW2( FAK(Degree) )), (ROUND_UP_POW2( FAK(Degree) ))>    MatrixU;
#endif

  enum {NUM=FAK(Degree)};       /// number of parameters for degree

  MatrixU model_;               /// regression data
  VectorU z_;                   /// result vector
#ifdef USE_MIN_MAX_RECHECK_
  float v_min_,v_max_;          /// min. z/max. z value of added data
#endif
  int occopied;                 /// mark

  /// constructor, sets everything to 0
  Param() {
    z_.fill(0);
    model_.fill(0);
  }

  /// init. with point
  inline void operator=(const Eigen::Vector3f &p) {
    if(pcl_isfinite(p(2))) {

#ifdef USE_MIN_MAX_RECHECK_
      v_min_=v_max_=p(2);
#endif

      for(int i=0; i<=Degree; i++)
        for(int j=0; j<=i; j++)
          z_(i*(i+1)/2+j) = std::pow(p(0),j)*std::pow(p(1),i-j);

      model_ = z_*z_.transpose();
      z_ *= p(2);
    }
    else {
#ifdef USE_MIN_MAX_RECHECK_
      v_min_=v_max_=0.f;
#endif
      z_.fill(0);
      model_.fill(0);
    }
  }

  ///add leave
  inline void operator+=(const Param<Degree> &p) {
    model_+=p.model_;
    z_+=p.z_;
#ifdef USE_MIN_MAX_RECHECK_
    if((p.v_min_!=0.f && p.v_min_<v_min_)||v_min_==0.f) v_min_=p.v_min_;
#ifndef CAMERA_ENSENSO
    if(p.v_max_>v_max_||v_max_==0.f) v_max_=p.v_max_;
#endif
#endif
  }

  inline float x() const {return model_(0,2)/model_(0,0);}      /// gets averaged x of current model (center point)
  inline float y() const {return model_(0,1)/model_(0,0);}      /// gets averaged y of current model (center point)
  inline float z() const {return z_(0)/model_(0,0);}            /// gets averaged z of current model (center point)
  
  template<typename Point>
  inline Point point() const {return Point(x(),y(),z());}
};

#ifdef QPPF_SPECIALIZATION_2
/**
 * leaf of quad-tree
 * contains data for regression of 6 parameters
 */
template< >
struct Param<2> {
#ifdef MIN_EIGEN_VECTOR
  typedef Eigen::Matrix<float, FAK(6), 1>    VectorU;
  typedef Eigen::Matrix<float, FAK(2), FAK(2)>    MatrixU;
#else
  typedef Eigen::Matrix<float, (ROUND_UP_POW2( FAK(2) )), 1>    VectorU;
  typedef Eigen::Matrix<float, (ROUND_UP_POW2( FAK(2) )), (ROUND_UP_POW2( FAK(2) ))>    MatrixU;
#endif

  enum {NUM=FAK(2)};

  MatrixU model_;        /// regression data
  VectorU z_;              /// result vector
#ifdef USE_MIN_MAX_RECHECK_
  float v_min_,v_max_;
#endif
  int occopied;             /// mark

  /// constructor, sets everything to 0
  Param(): model_(MatrixU::Zero()) ,z_(VectorU::Zero()){
    //for(int i=0; i<model_.rows(); i++)
    //{z_(i)=0; for(int j=0; j<model_.cols(); j++) model_(i,j)=0;}
  }

  /// init. with point
  inline void operator=(const Eigen::Vector3f &p) {
    if(pcl_isfinite(p(2))) {

#ifdef USE_MIN_MAX_RECHECK_
      v_min_=v_max_=p(2);
#endif
      float x=p(0);
      float x2=x*x;
      float y=p(1);
      float y2=y*y;

#if 0
      z_(0) = 1;
      z_(1) = x;
      z_(2) = x2;
      z_(3) = y;
      z_(4) = y2;
      z_(5) = x*y;

      model_ = z_*z_.transpose();
      z_ *= p(2);

#else

      z_(0) = p(2);
      z_(1) = p(2)*x;
      z_(2) = p(2)*x2;
      z_(3) = p(2)*y;
      z_(4) = p(2)*y2;
#ifndef DONT_USE_6TH
      z_(5) = p(2)*x*y;
#endif

      for(int i=0; i<6; i++) {
        float m=1;
        switch(i) {
          case 1: m=x;break;
          case 2: m=x2;break;
          case 3: m=y;break;
          case 4: m=y2;break;
#ifndef DONT_USE_6TH
          case 5: m=x*y;break;
#else
          case 5: m=0.f;break;
#endif
        }
        model_(i,0)=m;
        model_(i,1)=x*m;
        model_(i,2)=x2*m;
        model_(i,3)=y*m;
        model_(i,4)=y2*m;
#ifndef DONT_USE_6TH
        model_(i,5)=x*y*m;
#endif
      }
#endif
    }
    else {
#ifdef USE_MIN_MAX_RECHECK_
      v_min_=v_max_=0.f;
#endif
      z_.fill(0.f);
      model_.fill(0.f);
//      z_(0)=z_(1)=z_(2)=z_(3)=z_(4)=z_(5)=0.f;
//      for(int i=0; i<6; i++) for(int j=0; j<6; j++)
//        model_(i,j)=0.f;
    }
  }

  ///add leave
  inline void operator+=(const Param<2> &p) {
    model_+=p.model_;
    z_+=p.z_;
#ifdef USE_MIN_MAX_RECHECK_
    if((p.v_min_!=0.f && p.v_min_<v_min_)||v_min_==0.f) v_min_=p.v_min_;
    if(p.v_max_>v_max_||v_max_==0.f) v_max_=p.v_max_;
#endif
  }

  inline float x() const {return model_(0,1)/model_(0,0);}
  inline float y() const {return model_(0,3)/model_(0,0);}
  inline float z() const {return z_(0)/model_(0,0);}
  
  template<typename Point>
  inline Point point() const {return Point(x(),y(),z());}
};
#endif

/**
 * contains level of quad-tree
 * w: width
 * h: height
 * data: nodes/leaves
 */
template<int Degree>
struct ParamC {
  Param<Degree> *data;
  unsigned int w,h;

  ParamC(const ParamC &p):w(p.w),h(p.h) {
    data=new Param<Degree>[w*h];
    for(size_t i=0; i<w*h; i++)
      data[i]=p.data[i];
  }

  ParamC(unsigned int w, unsigned int h):w(w),h(h) {
    data=new Param<Degree>[w*h];
  }

  ~ParamC() {
    delete [] data;
  }
};

#endif /* PARAM_H_ */
