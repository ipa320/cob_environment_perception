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
