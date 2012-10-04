/*
 * surface_tri_spline.hpp
 *
 *  Created on: 01.10.2012
 *      Author: josh
 */



bool SurfaceTriSpline::TRIANGLE::update(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals, const std::vector<Eigen::Vector2f> &uv_pts)
{
  //update barycentric coordinates transformation
  _T_.col(0) = uv_pts[i_[0]]-uv_pts[i_[2]];
  _T_.col(1) = uv_pts[i_[1]]-uv_pts[i_[2]];
  _T_ = _T_.inverse().eval();

  //normal of tensor
  add_cross_ = ( pts[i_[1]]-pts[i_[0]] ).cross( pts[i_[2]]-pts[i_[0]] );
  add_cross_.normalize();

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

  add_cp_ = r*np + vp;
  add_cp_tp_(0) = ;
  add_cp_tp_(3) = add_cp_.dot(add_cross_);
  add_cp_+= pts[i_[0]];

  return pcl_isfinite(add_cp_.sum()+_T_.sum()+add_cp_tp_.sum());
}

void SurfaceTriSpline::TRIANGLE::getWeight(const Eigen::Vector3f &pt, Eigen::Matrix3f &w) const
{
  for(int i=0; i<3; i++) {
    w(i,0) = (pt(i)-1)*(pt(i)-1);
    w(i,1) = (pt(i)  )*(pt(i)-1);
    w(i,2) = (pt(i)  )*(pt(i)  );
  }
}

void SurfaceTriSpline::TRIANGLE::getWeightD1(const Eigen::Vector3f &pt, Eigen::Matrix3f &w) const
{
  for(int i=0; i<3; i++) {
    w(i,0) = (pt(i)-1)*(pt(i)-1);
    w(i,1) = (pt(i)  )*(pt(i)-1);
    w(i,2) = (pt(i)  )*(pt(i)  );
  }
}

Eigen::Vector4f SurfaceTriSpline::TRIANGLE::project2tensor(const Eigen::Vector3f &pt) const
{
  //pt in bayrcentric coordinates
  //weights for each side
  Eigen::Matrix3f w;
  getWeight(pt, w);

  Eigen::Vector4f r;
  for(int i=0; i<3; i++)
    r(i) = w.col(1)(i)*add_cp_tp_(i) + w.col(2)(i); //(second is 1)
  r(3) = add_cp_tp_(3)*w.col(1).sum();
  return r;
}


Eigen::Vector3f SurfaceTriSpline::TRIANGLE::project2world(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector3f> &pts) const
{
  //1. bayrcentric coordinates 2D -> 3D
  Eigen::Vector3f br;
  br.head<2>() = _T_*(pt-uv_pts_[i_[2]])
  br(2) = 1-br(0)-br(1);

  Eigen::Vector4f t = project2tensor(br);

  return (t(0)*pts[i_[0]] + t(1)*pts[i_[1]] + t(2)*pts[i_[2]]) + t(3)*add_cross_;

  /*
  //inside: br(0)>=0 && br(0)<1 && br(1)>=0 && br(1)<1 && br(2)>=0 && br(2)<1
   */
}

void SurfaceTriSpline::init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight)
{

}

/*
 * merging rules:
 *
 * generate merge pts + normals of all input data (including mid pts ???)
 * build topology with valid connections (TODO)
 * check
 */
