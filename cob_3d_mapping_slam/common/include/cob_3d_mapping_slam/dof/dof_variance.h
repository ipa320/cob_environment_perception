/*
 * dof_variance.h
 *
 *  Created on: 27.05.2012
 *      Author: josh
 */

#ifndef DOF_VARIANCE_H_
#define DOF_VARIANCE_H_

#include "euler.h"
#include <ros/time.h>

//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  /**
   * use recursive to add n sources
   */
  template <typename _SOURCE1, typename _SOURCE2>
  class DOF6_Source
  {
  public:
    typedef _SOURCE1 SOURCE1;
    typedef _SOURCE2 SOURCE2;
    typedef typename SOURCE1::TYPE TYPE;
    typedef Eigen::Matrix<TYPE,3,1> Vector;
    typedef boost::shared_ptr<SOURCE1> Source1_Ptr;
    typedef boost::shared_ptr<SOURCE2> Source2_Ptr;
    typedef boost::shared_ptr<const SOURCE1> Source1_ConstPtr;
    typedef boost::shared_ptr<const SOURCE2> Source2_ConstPtr;

  private:
    Source1_Ptr src1_;
    Source2_Ptr src2_;

  public:

    DOF6_Source(Source1_Ptr src1, Source2_Ptr src2)
    :src1_(src1),src2_(src2)
    {
      ROS_ASSERT(src1_);
      ROS_ASSERT(src2_);
    }

    DOF6_Source()
    :src1_(new SOURCE1),src2_(new SOURCE2)
    {
    }

    void deepCopy(const DOF6_Source &o)
    {
        src1_->deepCopy(*o.src1_);
        src2_->deepCopy(*o.src2_);
    }

    Source1_Ptr getSource1() {return src1_;}
    Source2_Ptr getSource2() {return src2_;}

    Source1_ConstPtr getSource1() const {return src1_;}
    Source2_ConstPtr getSource2() const {return src2_;}

    TYPE getRotationVariance() const
    {
      const TYPE C1 = src1_->getRotationVariance();
      const TYPE C2 = src2_->getRotationVariance();
      return C1 - C1*C1/(C1+C2);
    }

    TYPE getTranslationVariance() const
    {
      const TYPE C1 = src1_->getTranslationVariance();
      const TYPE C2 = src2_->getTranslationVariance();
      return C1 - C1*C1/(C1+C2);
    }

    EulerAngles<TYPE> getRotation() const
    {
      if(src1_->isRealSource() && src2_->isRealSource())
      {
        const EulerAngles<TYPE> X1 = src1_->getRotation();
        const EulerAngles<TYPE> X2 = src2_->getRotation();
        const TYPE C1 = src1_->getRotationVariance2();
        const TYPE C2 = src2_->getRotationVariance2();
        return X1 + C1/(C1+C2) * (X2-X1);
      }
      else if(src1_->isRealSource())
        return src1_->getRotation();
      else
        return src2_->getRotation();
    }

    Vector getTranslation() const
    {
      if(src1_->isRealSource() && src2_->isRealSource())
      {
        const Vector X1 = src1_->getTranslation();
        const Vector X2 = src2_->getTranslation();
        const TYPE C1 = src1_->getTranslationVariance2();
        const TYPE C2 = src2_->getTranslationVariance2();
        return X1 + C1/(C1+C2) * (X2-X1);
      }
      else if(src1_->isRealSource())
        return src1_->getTranslation();
      else
        return src2_->getTranslation();
    }

    void reset() {
      src1_->reset();
      src2_->reset();
    }

    void setTime(const double time_in_sec)
    {
      src1_->setTime(time_in_sec);
      src2_->setTime(time_in_sec);
    }

    inline bool isRealSource() const {return src1_->isRealSource()||src2_->isRealSource();}

    void setVariance(const TYPE Tvar, const Vector &tr, const TYPE Rvar, const EulerAngles<TYPE> &rot)
    {
      src1_->setVariance(Tvar, tr, Rvar, rot);
      src2_->setVariance(Tvar, tr, Rvar, rot);
    }

  };

  template <typename _SOURCE1, typename _SOURCE2>
  std::ostream &operator<<(std::ostream &os, const DOF6_Source<_SOURCE1,_SOURCE2> &s)
  {
    os<<"DOF6_Source\n\n";

    os<<"rot\n"<<s.getRotation()<<"\n";
    os<<"tr\n"<<s.getTranslation()<<"\n";
    os<<"var rot: "<<s.getRotationVariance()<<"\n";
    os<<"var tr:  "<<s.getTranslationVariance()<<"\n";

    os<< *s.getSource1()<<"\n";
    os<< *s.getSource2();

    return os;
  }

}



#endif /* DOF_VARIANCE_H_ */
