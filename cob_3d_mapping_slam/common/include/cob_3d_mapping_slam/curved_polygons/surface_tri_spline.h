/*
 * surface_tri_spline.h
 *
 *  Created on: 01.10.2012
 *      Author: josh
 */

#ifndef SURFACE_TRI_SPLINE_H_
#define SURFACE_TRI_SPLINE_H_



namespace Slam_Surface
{
  /**
   * topology consisting of triangles
   * each triangle is defined through 3 points and 3 normals at each point
   * --> 3 planes with the normal of the point and through the point
   * --> Intersection of the planes --> 4th control point
   * triangle builds plane for tensor product
   * outer points are 0 on tensor product, 4th control point has value z on x,y
   * B-Spline on tri.-plane
   */
  class SurfaceTriSpline : public Surface
  {

    struct TRIANGLE {
      size_t i_[3];
      Eigen::Vector3f add_cp_; //temporary control point, calculated from the normals
      Eigen::Vector3f add_cp_tp_; //temporary control point, calculated from add_cp_ on plane

      bool update(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals);
      Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const;
    };

    std::vector<Eigen::Vector3f> pts_, normals_;
    std::vector<TRIANGLE> triangles_;

  public:

    /// init with 6 parameters
    virtual void init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight);

    /// get implementation details
    virtual int getSurfaceType() const {return 3;}
    virtual const char *getName() const {return "Triangular B-Spline";}

    /// find nearest point to manifold
    virtual Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const ;

    /// project a 2D point to 3D
    virtual Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const ;

    /// get normal at 2D point
    virtual Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const ;

    /// merge parameters
    virtual float merge(const Surface &o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o);
    virtual float _merge(SurfaceNurbs o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o);

    /// transform basis
    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr);

    /// check form against
//    virtual bool fitsCurvature(const Surface &o, const float thr) const;

    /// calc approx. area
//    virtual float area() const {return nurbs_.area(0.0001f,20);}
  };

#include "impl/surface_tri_spline.hpp"
}



#endif /* SURFACE_TRI_SPLINE_H_ */
