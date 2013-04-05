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
  class Translation
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
      translation_ = translation_ + variance_*(variance_+r.variance_).inverse()*(r.translation-translation);
      variance_ = variance_ - variance_*(variance_+r.variance_).inverse()*variance_;
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
  };

  typedef Translation<float> Translationf;
  typedef Translation<double> Translationd;

#include "impl/translation.hpp"

}
