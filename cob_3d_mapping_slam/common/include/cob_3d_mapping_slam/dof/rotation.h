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
#pragma once

#include <eigen3/Eigen/Dense>

//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{


  template<typename TYPE>
  class RotationAxis
  {
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,3,1> Vector;
    typedef Eigen::Matrix<TYPE,2,1> Vector2;
    /** @todo: take faster one */
    typedef Eigen::EigenSolver<Matrix> EigenSolver;

    Matrix variance_;
    Matrix axis_;
    Vector var_;
#ifdef DEBUG_
    bool initialized_;
#endif

  public:
    RotationAxis()
#ifdef DEBUG_
    :initialized_(false)
#endif
    {
      axis_.fill(0);
    }
    RotationAxis(const Vector &n_old, const Vector &n_new, const Vector &var)
#ifdef DEBUG_
    :initialized_(false)
#endif
    {
      set(n_old,n_new,var);
    }

    void set(const Vector &n_old, const Vector &n_new, const Vector &var) {
#ifdef DEBUG_
      initialized_ = false;
#endif
      Matrix V,Q;
      Vector v,u,w;

      //Eigen composition: eigenvalues
      V.fill(0);
      V(0,0) = var(0);
      V(1,1) = var(1);
      V(2,2) = var(2);

      //Eigen composition: eigenvectors
      v=n_new-n_old;
      u=n_old+v/2;
      w=v.cross(u);

      v.normalize();
      u.normalize();
      w.normalize();

      variance_.col(0)=v;
      variance_.col(1)=u;
      variance_.col(2)=w;

      //Eigen composition: final
      variance_ = variance_*V*variance_.inverse();
    }

    /*!
     * updating covariance after "Merging Gaussian Distributions for Object Localization in Multi-Robot Systems"
     * \f$C'=C_1-C_1[C_1+C_2]^{-1}C_1\f$
     *
     */
    void operator+=(const RotationAxis &r)
    {
#ifdef DEBUG_
      initialized_ = false;
#endif
      variance_ = (variance_ - variance_*(variance_+r.variance_).inverse()*variance_);
    }

    /**
     * alpha can be resolved from rotation angles and normals as follows:
     * \f$cos \alpha = (n \cross r)(n' \cross r)\f$
     * using Lagrange-Identity follows:
     * \f$cos \alpha = (n \dot n')(r \dot r) - r(n'^Tn)r^T\f$
     */
    void calc() {
      EigenSolver solver(variance_);
/*
      std::cout << "The eigenvalues of A are:\n" << solver.eigenvalues() << std::endl;
      std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
          << "corresponding to these eigenvalues:\n"
          << solver.eigenvectors() << std::endl;
*/
      var_ = solver.eigenvalues().real();
      axis_= solver.eigenvectors().real();

#ifdef DEBUG_
      initialized_ = true;
#endif
    }

    inline const Matrix &getRotationAxis() const {
#ifdef DEBUG_
      ROS_ASSERT(initialized_);
#endif
      return axis_;
    }

    int getMainAxis() const {
      if(var_(0)>var_(1) && var_(0)>var_(2))
        return 0;
      else if(var_(1)>var_(2))
        return 1;
      return 2;
    }

    Vector getVariance() const {return var_;}
  };

  typedef RotationAxis<float> RotationAxisf;
  typedef RotationAxis<double> RotationAxisd;


  template<typename TYPE>
  class RotationAngle
  {
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,3,1> Vector;

    Matrix var_;

    const RotationAxis<TYPE> * const axis_;

  public:
    RotationAngle(const RotationAxis<TYPE> * const axis):axis_(axis) {
      var_.fill(0);
    }

    void add(const Vector &n_old, const Vector &n_new, const TYPE weight=1) {

      for(int i=0; i<3; i++) {
        Vector r = axis_->getRotationAxis().col(i);
        Vector a=r.cross(n_old), b=r.cross(n_new);
        a.normalize();
        b.normalize();
        const float alpha = std::acos( a.dot(b) );
        if(!pcl_isfinite(alpha))
          continue;
        //std::cout <<"alpha"<<i<<": "<< alpha<<"\n";

        var_.col(i)(0)+=weight;
        var_.col(i)(1)+=weight*alpha;
        var_.col(i)(2)+=weight*alpha*alpha;//todo: check
      }
    }

    Vector getAngles() const {
      Vector a;
      a(0) = var_.col(0)(1)/var_.col(0)(0);
      a(1) = var_.col(1)(1)/var_.col(1)(0);
      a(2) = var_.col(2)(1)/var_.col(2)(0);
      return a;
    }

    Vector getVariances() const {
      Vector a;
     /*TODO a(0) = var_.col(0)(1)/var_.col(0)(0);
      a(1) = var_.col(1)(1)/var_.col(1)(0);
      a(2) = var_.col(2)(1)/var_.col(2)(0);*/
      return a;
    }

  };

  typedef RotationAngle<float> RotationAnglef;
  typedef RotationAngle<double> RotationAngled;

  template<typename TYPE>
  class Rotation
  {
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,3,1> Vector;

    RotationAngle<TYPE> angle_;
    RotationAxis<TYPE> axis_;
    int num_;
  public:
    Rotation():
      angle_(&axis_), num_(0)
    {}

    void add1(const Vector &n_old, const Vector &n_new, const Vector &var)
    {
      if(num_==0)
        axis_.set(n_old,n_new,var);
      else
        axis_+=RotationAxis<TYPE>(n_old,n_new,var);
      ++num_;
    }

    void finish1() {
      axis_.calc();
    }

    void add2(const Vector &n_old, const Vector &n_new, const TYPE weight=1)
    {
      angle_.add(n_old,n_new, weight);
    }

    Matrix toRotationMatrix() const {
      const int axis=axis_.getMainAxis();

      std::cout <<"alpha: "<<angle_.getAngles()(axis)<<"\n";
      std::cout <<"r:\n"<<axis_.getRotationAxis().col(axis)<<"\n";
      std::cout <<"e:\n"<<axis_.getVariance()<<"\n";

      if(axis_.getRotationAxis().col(axis).squaredNorm()<0.01
          || !pcl_isfinite(axis_.getRotationAxis().col(axis).sum())
          || !pcl_isfinite(angle_.getAngles()(axis)))
        return Matrix::Identity();

      Eigen::AngleAxis<TYPE> aa(angle_.getAngles()(axis),axis_.getRotationAxis().col(axis));

      return aa.toRotationMatrix();
    }
  };

  typedef Rotation<float> Rotationf;
  typedef Rotation<double> Rotationd;



  template<typename TYPE>
  class Rotation2
  {
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,3,1> Vector;
    typedef Eigen::Matrix<TYPE,2,1> Vector2;
    /** @todo: take faster one */
    typedef Eigen::EigenSolver<Matrix> EigenSolver;

    Matrix variance_, covariance_;
    Matrix axis_;
    Vector var_;
    TYPE accumulated_weight_;
#ifdef DEBUG_
    bool initialized_;
#endif

  public:
    Rotation2()
    :accumulated_weight_(0)
#ifdef DEBUG_
    ,initialized_(false)
#endif
    {
      axis_.fill(0);
    }
    Rotation2(const Vector &n_old, const Vector &n_new, const Vector &var, const float weight):
      accumulated_weight_(0)
#ifdef DEBUG_
    ,initialized_(false)
#endif
    {
      set(n_old,n_new,var,weight);
    }

    void set(const Vector &n_old, const Vector &n_new, const Vector &var, const float weight) {
#ifdef DEBUG_
      initialized_ = false;
#endif
      Matrix V,Q;
      Vector v,u,w;

      //Eigen composition: eigenvalues
      V.fill(0);
      V(0,0) = var(0);
      V(1,1) = var(1);
      V(2,2) = var(2);

      //Eigen composition: eigenvectors
      v=n_new-n_old;
      u=n_old+v/2;
      w=v.cross(u);

      v.normalize();
      u.normalize();
      w.normalize();

      variance_.col(0)=v;
      variance_.col(1)=u;
      variance_.col(2)=w;

      //Eigen composition: final
      variance_ = variance_*V*variance_.inverse();

      //calc covariance
      covariance_ = weight*n_old*n_new.transpose();
      accumulated_weight_+=weight;
    }

    /*!
     * updating covariance after "Merging Gaussian Distributions for Object Localization in Multi-Robot Systems"
     * \f$C'=C_1-C_1[C_1+C_2]^{-1}C_1\f$
     *
     */
    void operator+=(const Rotation2 &r)
    {
#ifdef DEBUG_
      initialized_ = false;
#endif
      variance_ = (variance_ - variance_*(variance_+r.variance_).inverse()*variance_);

      accumulated_weight_ += r.accumulated_weight_;
      float alpha = r.accumulated_weight_/accumulated_weight_;

      covariance_ = (1.0-alpha)*(covariance_ + r.covariance_);
    }

    /**
     * alpha can be resolved from rotation angles and normals as follows:
     * \f$cos \alpha = (n \cross r)(n' \cross r)\f$
     * using Lagrange-Identity follows:
     * \f$cos \alpha = (n \dot n')(r \dot r) - r(n'^Tn)r^T\f$
     */
    void calc() {
      EigenSolver solver(variance_);
/*
      std::cout << "The eigenvalues of A are:\n" << solver.eigenvalues() << std::endl;
      std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
          << "corresponding to these eigenvalues:\n"
          << solver.eigenvectors() << std::endl;
*/
      var_ = solver.eigenvalues().real();
      axis_= solver.eigenvectors().real();

#ifdef DEBUG_
      initialized_ = true;
#endif
    }

    inline const Matrix &getRotationAxis() const {
#ifdef DEBUG_
      ROS_ASSERT(initialized_);
#endif
      return axis_;
    }

    int getMainAxis() const {
      if(var_(0)>var_(1) && var_(0)>var_(2))
        return 0;
      else if(var_(1)>var_(2))
        return 1;
      return 2;
    }

    Vector getVariance() const {return var_;}
  };

  typedef Rotation2<float> Rotation2f;
  typedef Rotation2<double> Rotation2d;

#include "impl/rotation.hpp"

}
