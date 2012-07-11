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
