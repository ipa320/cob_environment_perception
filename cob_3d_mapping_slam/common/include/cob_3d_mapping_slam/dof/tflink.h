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

#include <ros/console.h>
#include <ros/assert.h>
#include <eigen3/Eigen/Dense>

//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  template<typename INPUT>
  class TFLink
  {
  public:
    typedef typename INPUT::Scalar TYPE;
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,4,4> Matrix4;
    typedef Eigen::Matrix<TYPE,3,1> Vector;

  private:

    Matrix covariance_, variance_x_, variance_y_;
    Matrix translation_;
    Vector var_;

    Matrix rot_;
    Vector tr_;

    Vector var_x_, var_y_;
    INPUT sum_x_, sum_y_;
    TYPE rot_sum_, rot_var_, tr_var_, accumlated_weight_, accumlated_weight_t_;

    std::vector<Vector> corsA_, corsB_;
#ifdef DEBUG_
    bool initialized_;
#endif

  public:
    typedef boost::shared_ptr<TFLink<INPUT> > Ptr;

    /**
     * TFLinkObj can represent a point or a plane
     * a object can be reduced to one of these
     * needed is a vector from viewpoint to the next point of object
     */
    struct TFLinkObj
    {
      Matrix translation_M_;
      Vector next_point_;
      float length_;
      Vector rotation_n_;
      float weight_R_,weight_t_;
      bool plane_;

      /*
       * normal is vector from view point to mid of cube or next point on plane
       */
      TFLinkObj(const INPUT &normal,
                const bool plane,
                const bool norm=false, const TYPE wR=1., const TYPE wT=1.)
      :weight_R_(wR),weight_t_(wT), plane_(plane)
      {
        rotation_n_ = normal;

        if(plane_) //plane or point
        {
          length_ = rotation_n_.norm();
          if(norm)
          {
            next_point_.fill(0);
            translation_M_ = translation_M_.Zero(); /// \f$C=QVQ^{-1}\f$
          }
          else
          {
            translation_M_ = normal*normal.transpose()/normal.squaredNorm(); /// \f$C=QVQ^{-1}\f$
            next_point_ = normal;
          }
        }
        else {
          length_ = 1.f;
          next_point_ = normal;
          translation_M_ = translation_M_.Identity();
        }

      }
    };

    TFLink()
    {
      reset();
    }

    void deepCopy(const TFLink &o)
    {
      *this = o;
    }

    void reset()
    {
#ifdef DEBUG_
      initialized_ = false;
#endif
      rot_var_ = 10000; //depends on max. speed
      tr_var_ = 10000;  //depends on max. speed

      rot_ = Matrix::Identity();
      tr_.fill(0);
      accumlated_weight_t_ = accumlated_weight_ = 0;

      rot_sum_ = 0;
      covariance_.fill(0);
      sum_x_.fill(0);
      sum_y_.fill(0);

      translation_.fill(0);
      var_x_.fill(0);
      var_y_.fill(0);

      variance_x_.fill(0);
      variance_y_.fill(0);

      corsA_.clear();
      corsB_.clear();
    }

    void debug_print() {
      std::cout<<"TRANSLATION MATRIX\n"<<translation_<<"\n";
    }

    void operator()(const TFLinkObj &obj, const TFLinkObj &cor_obj);    /// adding objects
    TFLink operator+(const TFLink &o) const;    /// create chain of tf-links
    void operator+=(const TFLink &o);    /// add tf-links

    TFLink<INPUT> transpose() const;   /// returns inverse
    void finish();      /// calculate normalized covariance for rotation

    inline Matrix getRotation() const {return rot_;}
    inline Vector getTranslation() const {return tr_;}
    Matrix4 getTransformation() const;  /// returns rotaton+translation

    inline void setRotation(const Matrix &m)    {rot_=m;}
    inline void setTranslation(const Vector &m) {tr_=m;}

    TYPE getRotationVariance() const {return rot_var_;}
    TYPE getTranslationVariance() const {return tr_var_;}

    void check() const;

    inline Ptr makeShared () { return Ptr (new TFLink<INPUT> (*this)); }

    void setTime(const double time_in_sec)
    {
      //not used
    }

    inline bool isRealSource() const {return true;}

    void setVariance(const TYPE Tvar, const Vector &tr, const TYPE Rvar, const Matrix &rot)
    {
      //not used
    }

    TYPE getRotationVariance2() const
    {
      return getRotationVariance();
    }

    TYPE getTranslationVariance2() const
    {
      return getTranslationVariance();
    }

    bool isSet() const {return accumlated_weight_t_!=0;}

  };

  template<typename INPUT>
  std::ostream &operator<<(std::ostream &os, const TFLink<INPUT> &s)
  {

    os<<"TFLink\n\n";

    os<<"rot\n"<<s.getRotation()<<"\n";
    os<<"tr\n"<<s.getTranslation()<<"\n";
    os<<"var rot: "<<s.getRotationVariance()<<"\n";
    os<<"var tr:  "<<s.getTranslationVariance()<<"\n";

    return os;
  }

  typedef TFLink<Eigen::Vector3f> TFLinkvf;
  typedef TFLink<Eigen::Vector3d> TFLinkvd;

#include "impl/tflink.hpp"
}
