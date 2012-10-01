/*
 * surface_tri_spline.hpp
 *
 *  Created on: 01.10.2012
 *      Author: josh
 */



bool SurfaceTriSpline::TRIANGLE::update(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals)
{
  //check if data are valid
  float v1,v2;
  v1 = ( pts[i_[1]]-pts[i_[0]] ).dot( normals[i_[0]] );
  v2 = ( pts[i_[1]]-pts[i_[0]] ).dot( normals[i_[1]] );
  if(v1*v2<0) return false;
  v1 = ( pts[i_[2]]-pts[i_[0]] ).dot( normals[i_[0]] );
  v2 = ( pts[i_[2]]-pts[i_[0]] ).dot( normals[i_[2]] );
  if(v1*v2<0) return false;
  v1 = ( pts[i_[1]]-pts[i_[2]] ).dot( normals[i_[2]] );
  v2 = ( pts[i_[1]]-pts[i_[2]] ).dot( normals[i_[1]] );
  if(v1*v2<0) return false;

  //origin: pts[i_[0]]

  //intersection of 2 planes --> line
  Eigen::Vector3f np = normals[i_[0]].cross( normals[i_[1]] );
  float dot = normals[i_[0]].dot( normals[i_[1]] );
  float h2 = normals[i_[1]].dot( pts[i_[1]]-pts[i_[0]] );
  Eigen::Vector3f vp =
      ( (-h2*dot)*normals[i_[0]] + (h2)*normals[i_[1]] )/(1-dot*dot);
  //line: p = r*np + vp

  //intersection of last plane with line
  float r = (pts[i_[2]]-pts[i_[0]]-vp).dot(normals[i_[2]]) / (normals[i_[2]].dot(np));

  add_cp_ = r*np + vp + pts[i_[0]];

  add_cp_tp_

  return pcl_isfinite(add_cp_tp_.sum());
}

Eigen::Vector3f SurfaceTriSpline::TRIANGLE::project2world(const Eigen::Vector2f &pt) const
{
  Eigen::Vector2f px = 2*pt(0)*add_cp_tp_x_ - 2*pt(0)*pt(0)*add_cp_tp_x_;
  px(0) += pt(0)*pt(0);

  Eigen::Vector2f py = 2*pt(1)*add_cp_tp_y_ - 2*pt(1)*pt(1)*add_cp_tp_y_;
  py(0) += pt(1)*pt(1);
}


void SurfaceTriSpline::init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight)
{

}
