// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_MATH_H
#define NUKLEI_MATH_H

#include <nuklei/Common.h>
#include <nuklei/LinearAlgebraTypes.h>

#include <iostream>
#include <cmath>

namespace nuklei {

#define NUKLEI_RANGE_CHECK(value, low, high)\
{\
  if (! (low <= value && value <= high))\
  {\
    NUKLEI_WARN("Range Error:" << NUKLEI_NVP((low <= value && value <= high))\
             << "\n" << NUKLEI_NVP(high)\
             << "\n" << NUKLEI_NVP(value)\
             << "\n" << NUKLEI_NVP(low))\
  }\
}

  template<typename T> T ACos(T fValue)
  {
    if ( -(T)1.0 < fValue )
    {
      if ( fValue < (T)1.0 )
        return (T)std::acos((double)fValue);
      else
        return (T)0.0;
    }
    else
    {
      return M_PI;
    }
  }
  
  template<typename T> T FastACos(T fValue)
  {
    NUKLEI_FAST_ASSERT(-1-FLOATTOL < fValue && fValue < 1+FLOATTOL);
    if (fValue >= 1) return 0;
    else if (fValue >= 0)
      return nuklei_wmf::Math<coord_t>::FastInvCos0( fValue );
    else if (fValue > -1)
      return M_PI - nuklei_wmf::Math<coord_t>::FastInvCos0( -fValue );
    else return M_PI;
  }
  
  template<typename T> T ASin(T fValue)
  {
    if ( -(T)1.0 < fValue )
    {
      if ( fValue < (T)1.0 )
        return (T)std::asin((double)fValue);
      else
        return M_PI/2.0;
    }
    else
    {
      return -M_PI/2.0;
    }
  }
  
  template<typename T> T FastNegExp(T fValue)
  {
    NUKLEI_FAST_ASSERT(0-1e-6 < fValue);
    if (fValue < 0) fValue = 0;
    return nuklei_wmf::Math<coord_t>::FastNegExp3( fValue );
  }
  
  inline coord_t
  trivariateGaussian(const Vector3 &x, const Vector3 &m, const Matrix3 &cov,
                      const weight_t w = 1)
  {
    const Vector3 diff = x-m;
    const Matrix3 inv = cov.Inverse();
    const Vector3 tmp = inv * diff;
    double val = diff.Dot(tmp);  
    double ret = std::exp(-.5 * val);
    ret /= std::pow(2*M_PI, 3.0/2) * std::sqrt(cov.Determinant());
    return w*ret;
  }
  
  inline coord_t
  trivariateGaussianDistance(const Vector3 &x, const Vector3 &m,
                               const Matrix3 &cov,
                               const weight_t w = 1)
  {
    const Vector3 diff = x-m;
    const Matrix3 inv = cov.Inverse();
    const Vector3 tmp = inv * diff;
    double val = diff.Dot(tmp);  
    
    double ret = -.5 * val + std::log(w) - 3.0/2 * std::log(2*M_PI) -
    .5 * std::log(cov.Determinant());
    
    return -ret;
  }
  
  // relative float equality test
  bool rfe(const double a, const double b, double tol = 1e-6);
  
  // absolute float equality test
  inline bool afe(const double a, const double b, double tol = 1e-6)
  {
    // warning: !(std::fabs(a-b) <= tol) is not eq to (std::fabs(a-b) > tol) because of nan/inf
    return (std::fabs(a-b) <= tol);
  }
  
  
  template<typename T>
  T angularDistance(T a, T b)
  {
    T d = std::fabs( atan2( std::sin(a - b), std::cos(a - b) ) ) / M_PI;
    if (d > 1)
    {
      NUKLEI_FAST_ASSERT(afe(d, 1));
      d = 1;
    }
    return d;
  }
  
  double confluentHypergeometric1F1(const double a, const double b, const double x);
  double besselI1(const double x);

  
  
}

#endif

