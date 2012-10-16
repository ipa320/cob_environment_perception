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

  Eigen::Vector3f v;
  Eigen::Vector2f v2,r;
  Eigen::Matrix2f M;
  v2(0)=1;v2(1)=0;
  float l;

  for(int i=0; i<3; i++)
  {
    /*v=pts[i_[(1+i)%3]]-pts[i_[i]];
    l=v.norm();
    v/=l;

    v2(0)=l;

    M.row(1)(0) = v.dot(normals[i_[i]]);
    M.row(1)(1) = v.dot(normals[i_[(1+i)%3]]);

    M.row(0)(0) = std::sqrt( 1 - M.row(1)(0)*M.row(1)(0) );
    M.row(0)(1) = std::sqrt( 1 - M.row(1)(1)*M.row(1)(1) );

    r = M.inverse()*v2;

    if(!pcl_isfinite(r(0)))
      I_[i] = 0.5f*(pts[i_[(1+i)%3]]+pts[i_[i]]);
        //return false;
    else

    I_[i] = -M.row(1)(0)*r(0) * add_cross_ + (r(0)*pts[i_[(1+i)%3]] + r(1)*pts[i_[i]])/r.sum();

    std::cout<<"p\n"<<pts[i_[(1+i)%3]]<<"\n"<<pts[i_[i]]<<"\n";
    std::cout<<"n\n"<<normals[i_[(1+i)%3]]<<"\n"<<normals[i_[i]]<<"\n";
    std::cout<<"I\n"<<I_[i]<<"\n";

    std::cout<<"v\n"<<v<<"\n";
    std::cout<<"l\n"<<l<<"\n";
    std::cout<<"M\n"<<M<<"\n";
    std::cout<<"r\n"<<r<<"\n";*/

    Eigen::Matrix<float,3,2> A;
    A.col(0) = add_cross_.cross(pts[i_[(1+i)%3]]-pts[i_[i]]).cross(normals[i_[i]]);
    A.col(1) = -add_cross_.cross(pts[i_[i]]-pts[i_[(1+i)%3]]).cross(normals[i_[(1+i)%3]]);
    r = A.colPivHouseholderQr().solve(pts[i_[(1+i)%3]]-pts[i_[i]]).head<2>();

    I_[i] = r(0)*A.col(0) + pts[i_[i]];

    std::cout<<"p\n"<<pts[i_[(1+i)%3]]<<"\n"<<pts[i_[i]]<<"\n";
    std::cout<<"n\n"<<normals[i_[(1+i)%3]]<<"\n"<<normals[i_[i]]<<"\n";
    std::cout<<"I\n"<<I_[i]<<"\n";
    std::cout<<"I2\n"<<-r(1)*A.col(1) + pts[i_[(1+i)%3]]<<"\n";

    std::cout<<"r\n"<<r<<"\n";
    std::cout<<"A\n"<<A<<"\n";

    std::cout<<"d\n"<<(pts[i_[(1+i)%3]]-pts[i_[i]])<<"\n";
    std::cout<<"d1\n"<<A.col(0).dot(normals[i_[i]])<<"\n";
    std::cout<<"d2\n"<<A.col(1).dot(normals[i_[(1+i)%3]])<<"\n";
  }

  return true;

#if 0

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
  const float dot = normals[i_[0]].dot( normals[i_[1]] );
  const float h2 = normals[i_[1]].dot( pts[i_[1]]-pts[i_[0]] );
  const float dot2 = (1-dot*dot);
  if(dot2) {
    Eigen::Vector3f vp =
        ( (-h2*dot)*normals[i_[0]] + (h2)*normals[i_[1]] )/dot2;
    //line: p = r*np + vp

    //intersection of last plane with line
    float r = (pts[i_[2]]-pts[i_[0]]-vp).dot(normals[i_[2]]) / (normals[i_[2]].dot(np));

    add_cp_ = r*np + vp;

    ROS_ASSERT(pcl_isfinite(np.sum()));
    ROS_ASSERT(pcl_isfinite(vp.sum()));
    ROS_ASSERT(pcl_isfinite(r));
  }
  else {
    add_cp_ = (pts[i_[0]]+pts[i_[1]]+pts[i_[2]])/3-pts[i_[0]];
  }
  add_cp_tp_(3) = add_cp_.dot(add_cross_);
  add_cp_+= pts[i_[0]];
  add_cp_tp_(0) = (pts[i_[0]]-pts[i_[2]]).dot(add_cp_-pts[i_[2]]) / (pts[i_[0]]-pts[i_[2]]).squaredNorm();       //next point to line
  add_cp_tp_(1) = (pts[i_[1]]-pts[i_[2]]).dot(add_cp_-pts[i_[2]]) / (pts[i_[1]]-pts[i_[2]]).squaredNorm();
  add_cp_tp_(2) = (pts[i_[0]]-pts[i_[1]]).dot(add_cp_-pts[i_[1]]) / (pts[i_[0]]-pts[i_[1]]).squaredNorm();

  ROS_ASSERT(pcl_isfinite(add_cp_.sum()));
  ROS_ASSERT(pcl_isfinite(_T_.sum()));
  ROS_ASSERT(pcl_isfinite(add_cp_tp_.sum()));

  return pcl_isfinite(add_cp_.sum())&&pcl_isfinite(_T_.sum())&&pcl_isfinite(add_cp_tp_.sum());
#endif
}

void SurfaceTriSpline::TRIANGLE::getWeight(const Eigen::Vector3f &pt, Eigen::Matrix3f &w) const
{
  for(int i=0; i<3; i++) {
    w(i,0) =   (1-pt(i))*(1-pt(i));
    w(i,1) = 2*(pt(i)  )*(1-pt(i));
    w(i,2) =   (pt(i)  )*(pt(i)  );
  }
}

void SurfaceTriSpline::TRIANGLE::getWeightD1(const Eigen::Vector3f &pt, Eigen::Matrix3f &w) const
{
  ROS_ASSERT(0);//TODO
  for(int i=0; i<3; i++) {
    w(i,0) =   (pt(i)-1)*(pt(i)-1);
    w(i,1) = 2*(pt(i)  )*(pt(i)-1);
    w(i,2) =   (pt(i)  )*(pt(i)  );
  }
}

Eigen::Vector3f SurfaceTriSpline::TRIANGLE::triNurbsBasis(const Eigen::Vector3f &bc, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3) const
{
  return bc(0)*p1 + bc(1)*p2 + bc(2)*p3;
}


Eigen::Vector3f SurfaceTriSpline::TRIANGLE::project2world(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector2f> &uv_pts) const
{
  //1. bayrcentric coordinates 2D -> 3D
  Eigen::Vector3f br;
  br.head<2>() = _T_*(pt-uv_pts[i_[2]]);
  br(2) = 1-br(0)-br(1);

  std::cout<<_T_<<"\n";
  std::cout<<(pt-uv_pts[i_[2]])<<"\n\n";
  std::cout<<br<<"\n";
  std::cout<<br(0)*uv_pts[i_[0]]+br(1)*uv_pts[i_[1]]+br(2)*uv_pts[i_[2]]<<"\n\n";

  Eigen::Vector3f p1 = triNurbsBasis(br, pts[i_[0]], I_[0], I_[2]);
  Eigen::Vector3f p2 = triNurbsBasis(br, I_[0], pts[i_[1]], I_[1]);
  Eigen::Vector3f p3 = triNurbsBasis(br, I_[2], I_[1], pts[i_[2]]);

  return triNurbsBasis(br, p1,p2,p3);

  /*
  //inside: br(0)>=0 && br(0)<1 && br(1)>=0 && br(1)<1 && br(2)>=0 && br(2)<1
   */
}

bool SurfaceTriSpline::TRIANGLE::isIn(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &uv_pts) const
{
  //1. bayrcentric coordinates 2D -> 3D
  Eigen::Vector3f br;
  br.head<2>() = _T_*(pt-uv_pts[i_[2]]);
  br(2) = 1-br(0)-br(1);

  return br(0)>=0 && br(0)<=1 && br(1)>=0 && br(1)<=1 && br(2)>=0 && br(2)<=1;
}

void SurfaceTriSpline::init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight)
{
}

void SurfaceTriSpline::init(const PolynomialSurface *params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight)
{
  Eigen::Vector2f uv[4];
  uv[0](0)=uv[3](0) = min_x;
  uv[1](0)=uv[2](0) = max_x;
  uv[0](1)=uv[1](1) = min_y;
  uv[2](1)=uv[3](1) = max_y;

  for(int i=0; i<4; i++)
    addPoint(
        params->project2world(uv[i]),params->normalAt(uv[i]),uv[i]
    );

  addTriangle(0,1,2);
  addTriangle(2,3,0);
}

void SurfaceTriSpline::addPoint(
    const Eigen::Vector3f &p1, const Eigen::Vector3f &n1, const Eigen::Vector2f &uv1
)
{
  pts_.push_back(p1);
  uv_pts_.push_back(uv1);
  normals_.push_back(n1);
}

void SurfaceTriSpline::addTriangle(const size_t i1, const size_t i2, const size_t i3) {
  ROS_ASSERT(i1<pts_.size());
  ROS_ASSERT(i2<pts_.size());
  ROS_ASSERT(i3<pts_.size());

  triangles_.push_back( TRIANGLE(i1,i2,i3) );
  ROS_ASSERT( triangles_.back().update(pts_,normals_,uv_pts_) );
}

/// transform basis
void SurfaceTriSpline::transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr)
{
  for(size_t i=0; i<pts_.size(); i++) {
    pts_[i] = rot*pts_[i]+tr;
    normals_[i] = rot*normals_[i];
  }
  for(size_t i=0; i<triangles_.size(); i++)
    triangles_[i].transform(rot,tr);
}

Eigen::Vector3f SurfaceTriSpline::project2world(const Eigen::Vector2f &pt) const
{
  size_t next=0;
  float dis = std::numeric_limits<float>::max();

  for(size_t i=0; i<triangles_.size(); i++)
  {
    if(triangles_[i].isIn(pt,uv_pts_)) {
      return triangles_[i].project2world(pt,pts_,uv_pts_);
    }

    const float d = (uv_pts_[triangles_[i].i_[0]]-pt).squaredNorm();
    if(d<dis) {
      dis = d;
      next = i;
    }

  }

  return triangles_[next].project2world(pt,pts_,uv_pts_);

  ROS_ASSERT(0);
}

/*
 * merging rules:
 *
 * generate merge pts + normals of all input data (including mid pts ???)
 * build topology with valid connections (TODO)
 * check
 */
