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

  public:
    struct TRIANGLE {
      size_t i_[3];

      Eigen::Vector3f add_cross_; //temporary control point, calculated from the normals
      Eigen::Matrix2f _T_; //temporary inverse matrix for barycentric coordinates

      Eigen::Vector3f I_[3]; /// intersection of normal planes at line
      Eigen::Vector3f nI_[3]; /// intersection of normal planes at line

      Eigen::Vector3f pS_, nS_; /// apex point
      Eigen::Vector3f pb_[3];
      Eigen::Matrix2f _Tb_[3]; //temporary inverse matrix for barycentric coordinates

      TRIANGLE(const size_t i1, const size_t i2, const size_t i3) {
        i_[0] = i1;
        i_[1] = i2;
        i_[2] = i3;
      }

      bool update1(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals, const std::vector<Eigen::Vector2f> &uv_pts,
                  const Surface *surf=NULL);

      bool update2(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals, const std::vector<Eigen::Vector2f> &uv_pts,
                  const Surface *surf=NULL);

      void getWeight(const Eigen::Vector3f &pt, Eigen::Matrix3f &w) const;
      void getWeightD1(const Eigen::Vector3f &pt, Eigen::Matrix3f &w) const;
      Eigen::Vector3f triNurbsBasis(const Eigen::Vector3f &pt, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3) const;
      Eigen::Vector3f project2world(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector2f> &uv_pts) const;
      Eigen::Vector3f normalAt(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector2f> &uv_pts) const;
      void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {add_cross_=rot*add_cross_;}

      bool isIn(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &uv_pts) const;
    };

    std::vector<Eigen::Vector3f> pts_, normals_;
    std::vector<Eigen::Vector2f> uv_pts_;
    std::vector<TRIANGLE> triangles_;

    static int getLineID(const size_t i1, const size_t i2) {return (std::min(i1,i2)<<16)|(std::max(i1,i2));}

    void addTriangle(const size_t i1, const size_t i2, const size_t i3, const Surface *surf=NULL);
    void addPoint(
        const Eigen::Vector3f &p1, const Eigen::Vector3f &n1, const Eigen::Vector2f &uv1
        );

    std::vector<TRIANGLE> &getTriangles() {return triangles_;}

  public:

    /// init with 6 parameters
    virtual void init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight);
    virtual void init(const PolynomialSurface *params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight);

    /// get implementation details
    virtual int getSurfaceType() const {return 3;}
    virtual const char *getName() const {return "Triangular B-Spline";}

    /// find nearest point to manifold
    virtual Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const {return Eigen::Vector2f();}

    /// project a 2D point to 3D
    virtual Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const ;

    /// get normal at 2D point
    virtual Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const ;

    /// merge parameters
    virtual float merge(const Surface &o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) {return 0;}
    virtual float _merge(SurfaceNurbs o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) {return 0;}

    /// transform basis
    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr);

    /// check form against
    virtual bool fitsCurvature(const Surface &o, const float thr) const {return true;}

    /// calc approx. area
    virtual float area() const {return 1;}
  };

#include "impl/surface_tri_spline.hpp"
}



#endif /* SURFACE_TRI_SPLINE_H_ */
