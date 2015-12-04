// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_LINEAR_ALGEBRA_H
#define NUKLEI_LINEAR_ALGEBRA_H

#include <sstream>
#include <nuklei/Random.h>
#include <nuklei/BoostSerialization.h>
#include <nuklei/Common.h>
#include <nuklei/Math.h>

namespace nuklei {

  /** @brief Namespace containing linear algebra functions (and some other functions). */
  namespace la {

    inline
      Quaternion quaternionCopy(const Matrix3& m)
      {
        Quaternion q;
        q.FromRotationMatrix(m);
        q.Normalize();
        return q;
      }

    inline
      Matrix3 matrixCopy(const Quaternion &q)
      {
        Matrix3 m;
        q.ToRotationMatrix(m);
        m.Orthonormalize();
        return m;
      }


    inline
      Quaternion quaternionCopy(const Quaternion& q)
      {
        return q;
      }

    inline
      Matrix3 matrixCopy(const Matrix3 &m)
      {
        return m;
      }


    inline
      void copyRotation(Quaternion& q, const Matrix3& m)
      {
        q = quaternionCopy(m);
      }

    inline
      void copyRotation(Matrix3& m, const Quaternion& q)
      {
        m = matrixCopy(q);
      }

    inline
      void copyRotation(Quaternion& q, const Quaternion& q2)
      {
        q = q2;
      }

    inline
      void copyRotation(Matrix3& m, const Matrix3& m2)
      {
        m = m2;
      }

    inline
      GVector gVectorCopy(const GVector& v)
      {
        return v;
      }

    inline
      GVector gVectorCopy(const Vector3& v)
      {
        return GVector(3, v);
      }

    inline
      Vector3 vector3Copy(const Vector3& v)
      {
        return v;
      }

    inline
      Vector3 vector3Copy(const GVector& v)
      {
        NUKLEI_ASSERT(v.GetSize() == 3)
        return Vector3(v[0], v[1], v[2]);
      }
  }
  
  inline
  std::istream& operator>>(std::istream &in, Vector3 &l)
  {
    NUKLEI_ASSERT(in >> l.X() >> l.Y() >> l.Z());
    return in;
  }
  
  
  inline
  std::istream& operator>>(std::istream &in, Quaternion &q)
  {
    NUKLEI_ASSERT(in >> q.W() >> q.X() >> q.Y() >> q.Z());
    return in;
  }
  
  inline
  std::ostream& operator<<(std::ostream &out, const Quaternion &q)
  {
    NUKLEI_ASSERT(out << q.W() << ' ' << q.X() << ' ' << q.Y() << ' ' << q.Z());
    return out;
  }
  
  inline
  std::ostream& operator<<(std::ostream &out, const Matrix3 &m)
  {
    NUKLEI_ASSERT(out << la::quaternionCopy(m));
    return out;
  }
  
  inline
  std::istream& operator>>(std::istream &in, Matrix3 &m)
  {
    Quaternion q;
    NUKLEI_ASSERT(in >> q);
    m = la::matrixCopy(q);
    return in;
  }

  std::ostream& operator<<(std::ostream &out, const GMatrix &m);
  std::istream& operator>>(std::istream &in, GMatrix &m);

}


namespace nuklei {

  namespace la {


    inline
      Vector3 normalized(const Vector3 &v)
      {
        Vector3 unit = v;
        unit.Normalize();
        return unit;
      }

    inline
      Quaternion normalized(const Quaternion &q)
      {
        Quaternion n = q;
        n.Normalize();
        return n;
      }

    inline
      Matrix3 normalized(const Matrix3 &m)
      {
        Matrix3 n = m;
        n.Orthonormalize();
        return n;
      }

    inline
      void makeZero(Vector3 &v)
      {
        v = Vector3::ZERO;
      }

    inline
      void makeIdentity(Matrix3 &m)
      {
        m = Matrix3::IDENTITY;
      }

    inline
      void makeIdentity(Quaternion &q)
      {
        q = Quaternion::IDENTITY;
      }


    inline
      Vector3 min(const Vector3 &v1, const Vector3 &v2)
      {
        Vector3 minv;
        for (int i = 0; i < 3; ++i)
          minv[i] = std::min(v1[i], v2[i]);
        return minv;
      }


    inline
      Vector3 max(const Vector3 &v1, const Vector3 &v2)
      {
        Vector3 maxv;
        for (int i = 0; i < 3; ++i)
          maxv[i] = std::max(v1[i], v2[i]);
        return maxv;
      }

    inline
      Vector3 mean(const Vector3 &v1, const Vector3 &v2)
      {
        Vector3 mean;
        for (int i = 0; i < 3; ++i)
          mean[i] = ( v1[i] + v2[i] ) / 2.0;
        return mean;
      }



    inline
      Vector3 xAxis(const Quaternion &q)
      {
        typedef coord_t Real;
        
        Real fTy  = ((Real)2.0)*q[2];
        Real fTz  = ((Real)2.0)*q[3];
        Real fTwy = fTy*q[0];
        Real fTwz = fTz*q[0];
        Real fTxy = fTy*q[1];
        Real fTxz = fTz*q[1];
        Real fTyy = fTy*q[2];
        Real fTzz = fTz*q[3];
        
        Vector3 x((Real)1.0-(fTyy+fTzz),
                  fTxy+fTwz,
                  fTxz-fTwy);
        
        return normalized(x);
      }
    
    inline
      Vector3 xAxis(const Matrix3 &m)
      {
        return m.GetColumn(0);
      }


    inline
      coord_t numDrift(const Quaternion &q)
      {
        return std::fabs(q.Length()-1);
      }

    inline
      coord_t numDrift(const Matrix3 &m)
      {
        coord_t sum = 1;
        for (int i = 0; i < 3; ++i)
          sum += std::fabs(Vector3(m.GetColumn(i)).Dot(m.GetColumn((i+1)%3))) + std::fabs(Vector3(m.GetColumn(i)).Length() - 1);

        return sum-1;
      }

    inline
      void check(const Quaternion &q, const char* msg)
      {
        coord_t nd = numDrift(q);
        if (!(nd < FLOATTOL))
        {
          std::cout << msg << ": \n" << NUKLEI_NVP(q) << "\n" <<
            NUKLEI_NVP(numDrift(q)) << std::endl;
        }
        NUKLEI_ASSERT(nd < FLOATTOL);
      }

    inline
      void check(const Matrix3 &m, const char* msg)
      {
        coord_t nd = numDrift(m);
        if (!(nd < FLOATTOL))
        {
          std::cout << msg << ": \n" << NUKLEI_NVP(m) << "\n" <<
            NUKLEI_NVP(numDrift(m)) << std::endl;
        }
        NUKLEI_ASSERT(nd < FLOATTOL);
      }
    
    inline
      Quaternion inverseRotation(const Quaternion &q)
      {
        return q.Conjugate();
      }

    inline
      Matrix3 inverseRotation(const Matrix3 &m)
      {
        return m.Transpose();
      }


    inline
      Quaternion slerp(double c,
                       const Quaternion &q1,
                       const Quaternion &q2)
      {
        //fixme: check this slerp method
        // if c == 0, s = m1 (check that)
        Quaternion q;
 
        if (q1.Dot(q2) < 0)
          q.Slerp(c, q1, -q2);
        else
          q.Slerp(c, q1, q2);
        
        q.Normalize();
  
        return q;
      }

    inline
      Matrix3 slerp(double c,
                    const Matrix3 &m1,
                    const Matrix3 &m2)
      {
        return matrixCopy(slerp(c, quaternionCopy(m1), quaternionCopy(m2)));
      }


    inline
      Quaternion mean(const Quaternion &q1,
                      const Quaternion &q2)
      {
        return slerp(.5, q1, q2);
      }

    inline
      Matrix3 mean(const Matrix3 &m1,
                   const Matrix3 &m2)
      {
        return matrixCopy(mean(quaternionCopy(m1), quaternionCopy(m2)));
      }


    inline
    Quaternion so3FromS2(const Vector3 &w)
    {
      Vector3 u, v;
      Vector3::GenerateComplementBasis(u, v, w);

      Matrix3 m;
      coord_t angle = 2*M_PI*Random::uniform();
      Vector3 up =  std::cos(angle)*u + std::sin(angle)*v;
      Vector3 vp = -std::sin(angle)*u + std::cos(angle)*v;

      m.SetColumn(0, w);
      m.SetColumn(1, up);
      m.SetColumn(2, vp);

      return la::quaternionCopy(m);
    }

    void eigenDecomposition(Matrix3 &eVectors, Vector3& eValues, const Matrix3& sym);

    double determinant(const GMatrix &m);
    GMatrix inverse(const GMatrix &m);
    
    //Vector3 project(const Plane3& plane, const Vector3& point);
    
    
    /**
     * @addtogroup matrix_transfo
     * @{
     */
    

    /** @brief Returns @f$ Xy + x @f$. */
    inline Vector3 transform(const Vector3& x,
                             const Matrix3& X,
                             const Vector3& y)
    {
      return X*y + x;
    }
    
    /** @brief Returns @f$ Xy + x @f$. */
    inline Vector3 transform(const Vector3& x,
                             const Quaternion& X,
                             const Vector3& y)
    {
      return X.Rotate(y) + x;
    }
    
    /** @brief @f$ z = X y + x,  Z = X Y @f$. */
    inline void transform(Vector3& z, Matrix3& Z,
                          const Vector3& x, const Matrix3& X,
                          const Vector3& y, const Matrix3& Y)
    {
      z = transform(x, X, y);
      Z = X*Y;
    }
    
    /** @brief @f$ z = X y + x,  Z = X Y @f$. */
    inline void transform(Vector3& z, Quaternion& Z,
                          const Vector3& x, const Quaternion& X,
                          const Vector3& y, const Quaternion& Y)
    {
      z = transform(x, X, y);
      Z = X*Y;
    }
    
    /** @brief @f$ z = X y + x,  Z = X Y @f$. */
    inline void transform(Vector3& z, Vector3& Z,
                          const Vector3& x, const Matrix3& X,
                          const Vector3& y, const Vector3& Y)
    {
      z = transform(x, X, y);
      Z = transform(Vector3::ZERO, X, Y);
    }
    
    /** @brief @f$ z = X y + x,  Z = X Y @f$. */
    inline void transform(Vector3& z, Vector3& Z,
                          const Vector3& x, const Quaternion& X,
                          const Vector3& y, const Vector3& Y)
    {
      z = transform(x, X, y);
      Z = transform(Vector3::ZERO, X, Y);
    }

    /** @brief Returns @f$ X^T (z-x) @f$. */
    inline Vector3 project(const Vector3& x,
                           const Matrix3& X,
                           const Vector3& z)
    {
      return X.Transpose() * (z-x);
    }
    
    /** @brief Returns @f$ X^T (z-x) @f$. */
    inline Vector3 project(const Vector3& x,
                           const Quaternion& X,
                           const Vector3& z)
    {
      return X.Conjugate().Rotate(z-x);
    }
    
    /** @brief @f$ y = X^T (z-x), Y = X^T Z @f$. */
    inline void project(Vector3& y, Matrix3& Y,
                        const Vector3& x, const Matrix3& X,
                        const Vector3& z, const Matrix3& Z)
    {
      y = project(x, X, z);
      Y = X.TransposeTimes(Z);
    }
    
    /** @brief @f$ y = X^T (z-x), Y = X^T Z @f$. */
    inline void project(Vector3& y, Quaternion& Y,
                        const Vector3& x, const Quaternion& X,
                        const Vector3& z, const Quaternion& Z)
    {
      y = project(x, X, z);
      Y = X.Conjugate() * Z;
    }
    
    /** @brief @f$ y = X^T (z-x), Y = X^T Z @f$. */
    inline void project(Vector3& y, Vector3& Y,
                        const Vector3& x, const Matrix3& X,
                        const Vector3& z, const Vector3& Z)
    {
      y = project(x, X, z);
      Y = project(Vector3::ZERO, X, Z);
    }
    
    /** @brief @f$ y = X^T (z-x), Y = X^T Z @f$. */
    inline void project(Vector3& y, Vector3& Y,
                        const Vector3& x, const Quaternion& X,
                        const Vector3& z, const Vector3& Z)
    {
      y = project(x, X, z);
      Y = project(Vector3::ZERO, X, Z);
    }
    
    /** @brief @f$ x = z - Z Y^T y, X = Z Y^T @f$ */
    inline void transfoTo(Vector3& x, Matrix3& X,
                          const Vector3& y, const Matrix3& Y,
                          const Vector3& z, const Matrix3& Z)
      {
        X = Z.TimesTranspose(Y);
        x = z - X*y;
      }

    /** @brief @f$ x = z - Z Y^T y, X = Z Y^T @f$ */
    inline void transfoTo(Vector3& x, Quaternion& X,
                          const Vector3& y, const Quaternion& Y,
                          const Vector3& z, const Quaternion& Z)
      {
        X = Z * Y.Conjugate();
        x = z - X.Rotate(y);
      }

    /** @} */
    
    template<class R> void fromAngleAxisString(R &r, const std::string &angleAxis)
    {
      coord_t angle;
      Vector3 axis;
      std::istringstream ris(angleAxis);
      NUKLEI_ASSERT(ris >> angle);
      NUKLEI_ASSERT(ris >> axis);
      NUKLEI_ASSERT_AFE(axis.Length(), 1);
      axis = normalized(axis);
      r.FromAxisAngle(axis, angle);
      r = normalized(r);
    }
    
    template<class R> std::string toAngleAxisString(const R &r)
    {
      Vector3 axis;
      coord_t angle;
      r.ToAxisAngle(axis, angle);
      return stringify(angle) + " " + stringify(axis);
    }

  }

  inline coord_t
  multivariateGaussian(const GVector &x, const GVector &m, const GMatrix &cov,
                       const weight_t w = 1)
  {
    int d = x.GetSize();
    NUKLEI_FAST_DEBUG_ASSERT(d = m.GetSize());
    const GVector diff = x-m;
    const GMatrix inv = la::inverse(cov);
    const GVector tmp = inv * diff;
    double val = diff.Dot(tmp);  
    double ret = std::exp(-.5 * val);
    ret /= std::pow(2*M_PI, double(d)/2) * std::sqrt(la::determinant(cov));
    return w*ret;
  }
  
  
}

namespace boost {
  namespace serialization {

    template<class Archive>
      void serialize(Archive & ar, nuklei_wmf::Vector3<nuklei::coord_t> &v, const unsigned int version)
      {
        ar & boost::serialization::make_nvp("x", v.X());
        ar & boost::serialization::make_nvp("y", v.Y());
        ar & boost::serialization::make_nvp("z", v.Z());
      }

    template<class Archive>
      void serialize(Archive & ar, nuklei_wmf::Quaternion<nuklei::coord_t> &q, const unsigned int version)
      {
        ar & boost::serialization::make_nvp("w", q.W());
        ar & boost::serialization::make_nvp("x", q.X());
        ar & boost::serialization::make_nvp("y", q.Y());
        ar & boost::serialization::make_nvp("z", q.Z());
      }

    template<class Archive>
      void serialize(Archive & ar, nuklei_wmf::Matrix3<nuklei::coord_t> &m, const unsigned int version)
      {
        nuklei_wmf::Quaternion<nuklei::coord_t> q = nuklei::la::quaternionCopy(m);
        serialize(ar, q, version);
        nuklei::la::copyRotation(m, q);
      }


  } // namespace serialization
} // namespace boost



#endif
