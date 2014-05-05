/*
 * surface.h
 *
 *  Created on: 09.08.2012
 *      Author: josh
 */

#ifndef SURFACE_H_
#define SURFACE_H_

#include "form.h"
#include "polygon_merger.h"
#include <unsupported/Eigen/NonLinearOptimization>

namespace Slam_Surface
{

  class Surface
  {
  public:

    struct SWINDOW {
      float min_x, max_x, min_y, max_y;
    };


    virtual ~Surface() {}

    /// init with 6 parameters
    virtual void init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight) = 0;

    /// get implementation details
    virtual int getSurfaceType() const = 0;
    virtual const char *getName() const = 0;

    /// find nearest point to manifold (Newton)
    virtual Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const = 0;

    /// project a 2D point to 3D
    virtual Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const = 0;

    /// get normal at 2D point
    virtual Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const = 0;

    /// merge parameters
    virtual float merge(const Surface &o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) = 0;

    /// transform basis
    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) = 0;

    /// check form against
    virtual bool fitsCurvature(const Surface &o, const float thr) const = 0;

    /// calc approx. area
    virtual float area() const = 0;
  };

  class PolynomialSurface : public Surface
  {
    //from segmentation....
    Eigen::Matrix3f param_;
    Eigen::Matrix<float,3,2> proj2plane_;


    struct MyFunctor
    {
      const float a,b,c, d,e;
      const float x0,y0,z0;

      int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
      {
        // distance
        fvec(0) = std::pow((-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)+d*x(0)+e*x(1)),2)+std::pow(x(1)-y0,2)+std::pow(x(0)-x0,2);
        fvec(1) = 0;

//        std::cout<<"x\n"<<x<<"\n";
//        std::cout<<"dist "<<fvec(0)<<"\n";

        return 0;
      }

      int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
      {
        //Jacobian
        //TODO:
        fjac(0,0) = 2*(c*x(1)+2*a*x(0)+d)*(-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)+d*x(0)+e*x(1))+2*(x(0)-x0);
        fjac(0,1) = 2*(2*b*x(1)+c*x(0)+e)*(-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)+d*x(0)+e*x(1))+2*(x(1)-y0);
        fjac(1,0) = fjac(1,1) = 0;

//        std::cout<<"x\n"<<x<<"\n";
//        std::cout<<"fjac\n"<<fjac<<"\n";

        return 0;
      }

      int inputs() const { return 2; }
      int values() const { return 2; } // number of constraints
    };

    Eigen::Vector2f _nextPoint(const Eigen::Vector3f &v, Eigen::Vector3f p, const int depth=0) const;

  public:

    /// init with 6 parameters
    virtual void init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight);

    /// get implementation details
    virtual int getSurfaceType() const {return 1;}
    virtual const char *getName() const {return "polynomial surface";}

    /// find nearest point to manifold (Newton)
    virtual Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const ;

    /// project a 2D point to 3D
    virtual Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const ;

    /// get normal at 2D point
    virtual Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const ;

    /// get normal (2nd derivate) at 2D point
    virtual Eigen::Vector3f normalAt2(const Eigen::Vector2f &v) const ;

    /// merge parameters
    virtual float merge(const Surface &o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) {return 0.f;}

    /// transform basis
    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr);

    /// check form against
    virtual bool fitsCurvature(const Surface &o, const float thr) const;

    /// calc approx. area
    virtual float area() const {return 1.f;}
  };

#include "impl/surface.hpp"

}

#include "surface_nurbs.h"
#include "surface_tri_spline.h"


#endif /* SURFACE_H_ */
