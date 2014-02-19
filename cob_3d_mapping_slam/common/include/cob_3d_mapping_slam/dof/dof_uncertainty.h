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
 * dof_uncertainty.h
 *
 *  Created on: 31.05.2012
 *      Author: josh
 */

#ifndef DOF_UNCERTAINTY_H_
#define DOF_UNCERTAINTY_H_

#include "euler.h"


//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  /**
   * use recursive to add n sources
   */
  template <typename ROBOTPARAMETERS, typename TYPE>
  class DOF6_Uncertainty
  {
    typedef Eigen::Matrix<TYPE,3,1> Vector;
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,4,4> Matrix4;

    TYPE translation_speed_, rotation_speed_; /// speed per second
    double start_time_, end_time_;

    double timeDelta() const
    {
#ifdef _DEBUG
      ROS_ASSERT(end_time_>start_time_);
#endif
      return end_time_ - start_time_;
    }

    Vector tr_;
    TYPE var_rot_off_, var_tr_off_;
    EulerAngles<TYPE> rot_;

  public:

    DOF6_Uncertainty()
    :translation_speed_(ROBOTPARAMETERS::getMaxSpeedTranslationPerSecond()),
     rotation_speed_(ROBOTPARAMETERS::getMaxSpeedRotationPerSecond()),
     start_time_(std::numeric_limits<double>::max()),
     end_time_(std::numeric_limits<double>::min()),
     tr_(Vector::Zero()), var_rot_off_(0), var_tr_off_(0)
    {
    }

    void deepCopy(const DOF6_Uncertainty &o)
    {
      *this = o;
    }

    DOF6_Uncertainty transpose() const {
      DOF6_Uncertainty r = *this;
      Matrix4 M = getTF4().inverse();
      r.tr_ = M.col(3).head(3);
      r.rot_ = (EulerAngles<TYPE>)M.topLeftCorner(3,3);
      return r;
    }

    DOF6_Uncertainty operator+(const DOF6_Uncertainty &o) const    /// create chain of tf-links
    {
      DOF6_Uncertainty r = *this;
      r.var_rot_off_= var_rot_off_+ o.var_rot_off_;
      r.var_tr_off_ = var_tr_off_ + o.var_tr_off_;
      r.start_time_ = r.end_time_;
      Matrix4 M = getTF4()*o.getTF4();
      r.tr_ = M.col(3).head(3);
      r.rot_ = (EulerAngles<TYPE>)M.topLeftCorner(3,3);
      return r;
    }

    TYPE getRotationVariance() const
    {
      return timeDelta()*rotation_speed_+var_rot_off_;
    }

    TYPE getTranslationVariance() const
    {
      return timeDelta()*translation_speed_+var_tr_off_;
    }

    TYPE getRotationVariance2() const
    {
      return 10*getRotationVariance();
    }

    TYPE getTranslationVariance2() const
    {
      return 10*getTranslationVariance();
    }

    EulerAngles<TYPE> getRotation() const
    {
      return rot_;
    }

    Vector getTranslation() const
    {
      return tr_;
    }

    void reset()
    {
      tr_=Vector::Zero();
      rot_=Eigen::Matrix3f::Identity();
      var_rot_off_=var_tr_off_=0;
      start_time_ = std::numeric_limits<double>::max();
      end_time_ = std::numeric_limits<double>::min();
      std::cout<<"reset "<<start_time_<<"\n";
    }

    void setTime(const double time_in_sec)
    {
      std::cout<<"setTime "<<time_in_sec<<"\n";
      start_time_ = std::min(start_time_, time_in_sec);
      end_time_ = std::max(end_time_, time_in_sec);
    }

    void setVariance(const TYPE Tvar, const Vector &tr, const TYPE Rvar, const EulerAngles<TYPE> &rot)
    {
      var_tr_off_ = Tvar;
      var_rot_off_ = Rvar;
      rot_ = rot;
      tr_ = tr;
      start_time_ = end_time_;
    }

    Matrix4 getTF4() const {
      Matrix4 r=Matrix4::Identity();
      r.topLeftCorner(3,3) = (Eigen::Matrix3f)getRotation();
      r.col(3).head(3)     = getTranslation();
      return r;
    }

    inline bool isRealSource() const {return true;}

    inline double getStartTime() const {return start_time_;}
    inline double getEndTime() const {return end_time_;}

  };

  template <typename ROBOTPARAMETERS, typename TYPE>
  std::ostream &operator<<(std::ostream &os, const DOF6_Uncertainty<ROBOTPARAMETERS,TYPE> &s)
  {

    os<<"DOF6_Uncertainty\n\n";

    os<<"rot\n"<<s.getRotation()<<"\n";
    os<<"tr\n"<<s.getTranslation()<<"\n";
    os<<"var rot: "<<s.getRotationVariance()<<"\n";
    os<<"var tr:  "<<s.getTranslationVariance()<<"\n";
    os<<"start: "<<s.getStartTime()<<"\n";
    os<<"end:   "<<s.getEndTime()<<"\n";

    return os;
  }

}


#endif /* DOF_UNCERTAINTY_H_ */
