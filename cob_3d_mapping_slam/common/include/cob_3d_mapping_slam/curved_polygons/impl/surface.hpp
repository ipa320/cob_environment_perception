/*
 * surface.hpp
 *
 *  Created on: 09.08.2012
 *      Author: josh
 */


void PolynomialSurface::init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight)
{
  param_.col(0) = Eigen::Vector3f::Zero();
  param_.col(0)(2) = params[0];

  param_.col(1)(0)=params[1];
  param_.col(1)(1)=params[3];
  param_.col(1)(2)=0;

  param_.col(2)(0)=params[2];
  param_.col(2)(1)=params[4];
  param_.col(2)(2)=params[5];

  Eigen::Vector3f x,y;
  x(0)=y(1)=1.f;
  x(1)=x(2)=y(0)=y(2)=0.f;

  proj2plane_.col(0)=x;
  proj2plane_.col(1)=y;
}

Eigen::Vector3f PolynomialSurface::project2world(const Eigen::Vector2f &pt) const {
  Eigen::Vector3f pt2;
  pt2(0)=pt(0)*pt(0);
  pt2(1)=pt(1)*pt(1);
  pt2(2)=pt(0)*pt(1);

  return param_.col(0) + proj2plane_*pt + proj2plane_.col(0).cross(proj2plane_.col(1)) * (pt2.dot(param_.col(2)) + pt.dot(param_.col(1).head<2>()));
}

Eigen::Vector3f PolynomialSurface::normalAt(const Eigen::Vector2f &v) const {
  Eigen::Vector3f r;

  r(0) = -(param_.col(1)(0) + 2*v(0)*param_.col(2)(0)+v(1)*param_.col(2)(2));
  r(1) = -(param_.col(1)(1) + 2*v(1)*param_.col(2)(1)+v(0)*param_.col(2)(2));
  r(2) = 1;

  r.normalize();

  Eigen::Matrix3f M;
  M.col(0) = proj2plane_.col(0);
  M.col(1) = proj2plane_.col(1);
  M.col(2) = proj2plane_.col(0).cross(proj2plane_.col(1));

  return M*r;
}

Eigen::Vector3f PolynomialSurface::normalAt2(const Eigen::Vector2f &v) const {
  Eigen::Vector3f r;

  r(0) = -(2*param_.col(2)(0));
  r(1) = -(2*param_.col(2)(1));
  r(2) = 0;

  r(0) = 0;
  r(1) = 0;
  r(2) = 1;

  //r.normalize();

  Eigen::Matrix3f M;
  M.col(0) = proj2plane_.col(0);
  M.col(1) = proj2plane_.col(1);
  M.col(2) = proj2plane_.col(0).cross(proj2plane_.col(1));

  return M*r;
}

void PolynomialSurface::transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr)
{
  param_.col(0) = rot * param_.col(0)+tr;

  proj2plane_.col(0) = rot * proj2plane_.col(0);
  proj2plane_.col(1) = rot * proj2plane_.col(1);
}

bool PolynomialSurface::fitsCurvature(const Surface &o, const float thr) const
{
  ROS_ASSERT(getSurfaceType()==getSurfaceType());

  return param_.col(2).cross(((PolynomialSurface*)&o)->param_.col(2)).squaredNorm() < thr*thr;
}

/// find nearest point to manifold (LM)
Eigen::Vector2f PolynomialSurface::_nextPoint(const Eigen::Vector3f &v, Eigen::Vector3f p, const int depth) const
{
  Eigen::VectorXf r(2);
  r = v.head<2>();
  MyFunctor functor={param_.col(2)(0),param_.col(2)(1),param_.col(2)(2),
                     param_.col(1)(0),param_.col(1)(1),
                     p(0),p(1),p(2)};
  Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);
  lm.parameters.maxfev = 50; //not to often (performance)
  lm.minimize(r);

  return r;
}

/// find nearest point to manifold (Newton)
Eigen::Vector2f PolynomialSurface::nextPoint(const Eigen::Vector3f &v) const
{
  Eigen::Vector3f p;

  p(0) = (v-param_.col(0)).dot(proj2plane_.col(0))/proj2plane_.col(0).squaredNorm();
  p(1) = (v-param_.col(0)).dot(proj2plane_.col(1))/proj2plane_.col(1).squaredNorm();
  p(2) = (v-param_.col(0)).dot(proj2plane_.col(0).cross(proj2plane_.col(1)));

  Eigen::Vector2f r = _nextPoint(p, p);

  float e1 = (v-project2world(p.head<2>())).norm();
  float e2 = (v-project2world(r)).norm();

  if(e1+0.001f<e2)
  {
    ROS_WARN("e1>=e2 %f %f",e1,e2);
    return p.head<2>();
  }

  return r;
}
