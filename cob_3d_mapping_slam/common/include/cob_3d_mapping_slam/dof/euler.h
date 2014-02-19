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
 * euler.h
 *
 *  Created on: 26.05.2012
 *      Author: josh
 */

#ifndef EULER_H_
#define EULER_H_


#include <eigen3/Eigen/Dense>

//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  /**
   * converts rotation matrix to Euler angles and back
   */
  template<typename TYPE>
  class EulerAngles
  {
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,3,1> Vector;

    TYPE alpha_, beta_, gamma_;
  public:
    EulerAngles():
      alpha_(0), beta_(0), gamma_(0)
    {
    }

    EulerAngles(const Matrix &R)
    {
      from(R);
    }
    EulerAngles(const TYPE alpha, const TYPE beta, const TYPE gamma):
      alpha_(alpha), beta_(beta), gamma_(gamma)
    {
    }

    /// after Computing Euler angles from a rotation matrix (Gregory G. Slabaugh)
    void from(const Matrix &R) {
      if(std::abs(R(2,0))!=1)
      {
        beta_ = -std::asin(R(2,0));
        const TYPE t=std::cos(beta_);
        gamma_= std::atan2(R(2,1)/t, R(2,2)/t);
        alpha_= std::atan2(R(1,0)/t, R(0,0)/t);
      }
      else
      {
        alpha_ = 0;
        if(R(2,0)==-1)
        {
          beta_ = M_PI/2;
          gamma_= std::atan2(R(0,1),R(0,2));
        }
        else
        {
          beta_ = -M_PI/2;
          gamma_= std::atan2(-R(0,1),-R(0,2));
        }
      }
    }

    Matrix toRotMat() const {
      Matrix R;

      R(0,0)=std::cos(beta_)*std::cos(alpha_);
      R(0,1)=std::sin(gamma_)*std::sin(beta_)*std::cos(alpha_)-std::cos(gamma_)*std::sin(alpha_);
      R(0,2)=std::cos(gamma_)*std::sin(beta_)*std::cos(alpha_)+std::sin(gamma_)*std::sin(alpha_);

      R(1,0)=std::cos(beta_)*std::sin(alpha_);
      R(1,1)=std::sin(gamma_)*std::sin(beta_)*std::sin(alpha_)+std::cos(gamma_)*std::cos(alpha_);
      R(1,2)=std::cos(gamma_)*std::sin(beta_)*std::sin(alpha_)-std::sin(gamma_)*std::cos(alpha_);

      R(2,0)=-std::sin(beta_);
      R(2,1)=std::cos(beta_)*std::sin(gamma_);
      R(2,2)=std::cos(beta_)*std::cos(gamma_);

      return R;
    }

    inline operator Matrix() const { return toRotMat(); }

    inline EulerAngles operator-(const EulerAngles &o) const {
      return EulerAngles(alpha_-o.alpha_, beta_-o.beta_, gamma_-o.gamma_);
    }

    inline EulerAngles operator+(const EulerAngles &o) const {
      return EulerAngles(alpha_+o.alpha_, beta_+o.beta_, gamma_+o.gamma_);
    }

    inline TYPE norm() const
    {
      return std::sqrt( alpha_*alpha_ + beta_*beta_ + gamma_*gamma_ );
    }

    inline Vector getVector() const {
      Vector v;
      v(0) = alpha_;
      v(1) = beta_;
      v(2) = gamma_;
      return v;
    }

    template<typename TTYPE>
    friend EulerAngles<TTYPE> operator*(const TTYPE m, const EulerAngles<TTYPE> &o);

    template<typename TTYPE>
    friend std::ostream &operator<<(std::ostream &os, const EulerAngles<TTYPE> &o);
  };

  template<typename TYPE>
  inline EulerAngles<TYPE> operator*(const TYPE m, const EulerAngles<TYPE> &o) {
    return EulerAngles<TYPE>(m*o.alpha_, m*o.beta_, m*o.gamma_);
  }

  template<typename TTYPE>
  inline std::ostream &operator<<(std::ostream &os, const EulerAngles<TTYPE> &o) {
    os<<"alpha: "<<o.alpha_<<"\n";
    os<<"beta:  "<<o.beta_<<"\n";
    os<<"gamma: "<<o.gamma_<<"\n";
    return os;
  }

  typedef EulerAngles<float> EulerAnglesf;
  typedef EulerAngles<double> EulerAnglesd;
}


#endif /* EULER_H_ */
