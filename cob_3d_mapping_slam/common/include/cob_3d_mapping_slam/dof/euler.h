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
