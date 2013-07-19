/*
 * surface_nurbs.h
 *
 *  Created on: 10.08.2012
 *      Author: josh
 */

#ifndef SURFACE_NURBS_H_
#define SURFACE_NURBS_H_

#include <nurbsS_sp.h>
#include <pcl/common/pca.h>

namespace Slam_Surface
{
  class SurfaceNurbs : public Surface
  {
    typedef PlNurbsSurfaceSPf SURFACE;
    SURFACE nurbs_;
    MatrixRTf weights_;

    //    Eigen::Vector3f plane_x_, plane_y_;
    float c_x_m_, c_x_o_, c_y_m_, c_y_o_;       /// offset correction

    virtual Eigen::Vector2f correct(const Eigen::Vector2f &v) const ;
    virtual Eigen::Vector3f _normalAt(const Eigen::Vector2f &v) const ;


    // for LM
    struct Functor {
      PLib::Point3Df _pt_;
      const SURFACE &nurbs_;

      Functor(const SURFACE &nurbs):nurbs_(nurbs)
      {}

      int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
      {
        // distance
        fvec(0) = norm2(_pt_ - nurbs_.pointAt(x(0),x(1)));
        fvec(1) = 0;

        std::cout<<"x\n"<<x<<"\n";
        std::cout<<"dist "<<fvec(0)<<"\n";

        //        std::cout<<"fvec\n"<<fvec<<std::endl;
        return 0;
      }

      int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
      {
        //Jacobian
        Matrix_Point3Df ders;
        nurbs_.deriveAt(x(0),x(1), 1, ders);
        PLib::Point3Df p = nurbs_.pointAt(x(0),x(1));
        fjac(0,0) = dot(ders(1,0),(_pt_-p));
        fjac(0,1) = dot(ders(0,1),(_pt_-p));
        fjac(1,0) = fjac(1,1) = 0;

        std::cout<<"ders\n"<<ders<<"\n";
        std::cout<<"fjac\n"<<fjac<<"\n";
        return 0;
      }

      int inputs() const { return 2; }
      int values() const { return 2; } // number of constraints
    };

    int myprojectOn(const SURFACE &s, const Eigen::Vector3f &p, float &u, float &v, const int maxIt) const;

    /// find nearest point to manifold
    Eigen::Vector2f _nextPoint(const Eigen::Vector3f &v) const ;


    struct PT_GRID {
      int w,h;
      std::vector<Eigen::Vector3f> grid;

      PT_GRID(const SURFACE &s, const int sampling) {
        w = s.ctrlPnts().rows()*sampling;
        h = s.ctrlPnts().cols()*sampling;
        grid.resize(w*h);
      }

      Eigen::Vector3f &operator()(int i, int j) {return grid[i*h+j];}
    };
    virtual float _merge2(SurfaceNurbs o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o);

  public:

    /// init with 6 parameters
    virtual void init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight);

    /// get implementation details
    virtual int getSurfaceType() const {return 2;}
    virtual const char *getName() const {return "NURBS";}

    /// find nearest point to manifold
    virtual Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const ;

    /// project a 2D point to 3D
    virtual Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const ;
    virtual Eigen::Vector3f _project2world(const Eigen::Vector2f &pt) const ;

    /// get normal at 2D point
    virtual Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const ;

    /// merge parameters
    virtual float merge(const Surface &o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o);
    virtual float _merge(SurfaceNurbs o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o);

    /// transform basis
    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr);

    /// check form against
    virtual bool fitsCurvature(const Surface &o, const float thr) const;

    const SURFACE &getNurbs() const {return nurbs_;}

    /// calc approx. area
    virtual float area() const {return nurbs_.area(0.0001f,20);}
  };

#include "impl/surface_nurbs.hpp"
}

#endif /* SURFACE_NURBS_H_ */
