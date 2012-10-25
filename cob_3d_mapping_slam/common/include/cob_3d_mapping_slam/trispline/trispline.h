/*
 * trispline.h
 *
 *  Created on: 25.10.2012
 *      Author: josh
 */

#ifndef TRISPLINE_H_
#define TRISPLINE_H_

#include <Eigen/Core>

namespace ParametricSurface {

  template<size_t IndexA, size_t IndexB, size_t IndexC>
  class TriangleInst
  {
  protected:
    Eigen::Vector3f* pts_;

  public:
    TriangleInst(Eigen::Vector3f* pts_):
      pts_(pts_)
    {}

    inline Eigen::Vector3f tri(const Eigen::Vector3f &bc) {
      return ptA()*bc(0) + ptB()*bc(1) + ptC()*bc(2);
    }

    inline Eigen::Vector3f normal() {
      Eigen::Vector3f r = (ptB()-ptA()).cross(ptC()-ptA());
      r.normalize();
      return r;
    }

    inline Eigen::Vector3f &ptA() {return pts_[IndexA];}
    inline Eigen::Vector3f &ptB() {return pts_[IndexB];}
    inline Eigen::Vector3f &ptC() {return pts_[IndexC];}

  };

  template<int RowSize, int Index>
  class TriangleIndex {
  public:
    enum {a=(Index-1), b=(Index), c=(Index+RowSize)};
  };

  template<int RowSize, int Index>
  class Triangle
  {

    TriangleInst<TriangleIndex<RowSize,Index>::a, TriangleIndex<RowSize,Index>::b, TriangleIndex<RowSize,Index>::c > b;
    Triangle<RowSize, Index-1> tri_;

  public:

    Triangle(Eigen::Vector3f* pts_):
      b(pts_), tri_(pts_) {}

  };

  template<int RowSize>
  class Triangle<RowSize,0>
  {
    TriangleInst<TriangleIndex<RowSize,0>::a, TriangleIndex<RowSize,0>::b, TriangleIndex<RowSize,0>::c > b;
    Triangle<RowSize-1, RowSize-2> tri_;

  public:

    Triangle(Eigen::Vector3f* pts_):
      b(pts_), tri_(pts_+RowSize) {}

  };

  template<>
  class Triangle<1,0>
  {
  public:

    TriangleInst<TriangleIndex<1,0>::a, TriangleIndex<1,0>::b, TriangleIndex<1,0>::c > b;

    Triangle(Eigen::Vector3f* pts_): b(pts_) {}

  };

  template<int Order>
  class TriSpline
  {
    Eigen::Vector3f pts_[(Order+1)*(Order+2)/2];

    Triangle<Order,Order-1> tris_;

  public:
    TriSpline():tris_(pts_) {}
  };

}

#endif /* TRISPLINE_H_ */
