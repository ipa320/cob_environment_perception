/*
 * surface_nurbs.hpp
 *
 *  Created on: 10.08.2012
 *      Author: josh
 */


#include "../debug_interface.h"


void SurfaceNurbs::init(const boost::array<float, 6> &params, const float min_x, const float max_x, const float min_y, const float max_y, const float weight)
{
  //  ROS_ERROR("%f %f",min_x,max_x);

  ROS_ASSERT(min_x<=max_x);
  ROS_ASSERT(min_y<=max_y);

  //  float sx, sy;
  //  if(std::abs(params[5])<0.0001f) {
  //    sx=-params[1]/(2*params[2]);
  //    sy=-params[3]/(2*params[4]);
  //  }
  //  else {
  //    sy= ((2*params[2]*params[3])/params[5]-params[1]) /
  //        ( params[5]-(4*params[2]*params[4]/params[5]));
  //
  //    sx=-(params[1]+params[5]*sy)/(2*params[2]);
  //  }

  Matrix_Point3Df Pts(3,3);
  for(int i=0;i<3;++i){
    for(int j=0;j<3;++j){
      const float x = min_x + (max_x-min_x)*(i*0.5f);
      const float y = min_y + (max_y-min_y)*(j*0.5f);
      Pts(i,j) = PlPoint3Df(x, y,
                            params[0] + x*params[1] + x*x*params[2] + y*params[3] + y*y*params[4] + x*y*params[5]
      ) ;
    }
  }
  nurbs_.globalInterp(Pts,2,2);

  weights_.resize(3,3);
  weights_.reset(weight);

  c_x_m_ = 1/(max_x-min_x);
  c_x_o_ = -min_x;
  c_y_m_ = 1/(max_y-min_y);
  c_y_o_ = -min_y;

#ifdef DEBUG_OUT_
  ROS_INFO("DEGREE %d %d",nurbs_.ctrlPnts().cols(),nurbs_.ctrlPnts().rows());
#endif

  //validate
//  for(int i=0;i<3;++i){
//    for(int j=0;j<3;++j){
//      Eigen::Vector2f p;
//      p(0)=i*0.5f;
//      p(1)=j*0.5f;
//      const float x = min_x + (max_x-min_x)*(i*0.5f);
//      const float y = min_y + (max_y-min_y)*(j*0.5f);
//      Eigen::Vector3f v;
//      v(0)=x;
//      v(1)=y;
//      v(2)=params[0] + x*params[1] + x*x*params[2] + y*params[3] + y*y*params[4] + x*y*params[5];
//      if(v.norm()<20) {
//      float d=(_project2world(p)-v).norm();
//      std::cout<<"dist "<<d<<std::endl;
//      std::cout<<"n\n"<<_project2world(p)<<std::endl;
//      std::cout<<"p\n"<<v<<std::endl;
//      std::cout<<"np\n"<<_nextPoint(v)<<std::endl;
//      d=(_project2world(_nextPoint(v))-v).norm();
//      std::cout<<"dist "<<d<<std::endl;
//      ROS_ASSERT(d<0.05f);
//      }
//    }
//  }

  //nurbs_.writeVRML97("torus.wrl");

  //  Eigen::Vector2f p1, p2;
  //
  //  p1(0) = min_x;
  //  p1(1) = min_y;
  //
  //  p2=p1;
  //  p2(0) = max_x;
  //  plane_x_ = project2world(p2)-project2world(p1);
  //
  //  p2=p1;
  //  p2(1) = max_y;
  //  plane_y_ = project2world(p2)-project2world(p1);
  //
  //  plane_x_ /= plane_x_.squaredNorm();
  //  plane_y_ /= plane_y_.squaredNorm();
}

Eigen::Vector2f SurfaceNurbs::correct(const Eigen::Vector2f &pt) const {
  Eigen::Vector2f r;
  r(0) = c_x_m_*(pt(0) + c_x_o_);
  r(1) = c_y_m_*(pt(1) + c_y_o_);
  return r;
}

Eigen::Vector3f SurfaceNurbs::project2world(const Eigen::Vector2f &pt) const {
  return _project2world(correct(pt));
}

Eigen::Vector3f SurfaceNurbs::_project2world(const Eigen::Vector2f &pt) const {
  Eigen::Vector3f r;
  PlPoint3Df p = nurbs_.pointAt(pt(0),pt(1));
  r(0) = p.x();
  r(1) = p.y();
  r(2) = p.z();
  return r;
}

Eigen::Vector3f SurfaceNurbs::normalAt(const Eigen::Vector2f &v) const {
  return _normalAt(correct(v));
}

Eigen::Vector3f SurfaceNurbs::_normalAt(const Eigen::Vector2f &v) const {
  Eigen::Vector3f r;
  PlMatrix_Point3Df ders;
  nurbs_.deriveAt(v(0),v(1),1,ders);
  PlPoint3Df p = crossProduct(ders(0,1),ders(1,0));
  r(0) = p.x();
  r(1) = p.y();
  r(2) = p.z();
  r.normalize();
  return -r;
}

void SurfaceNurbs::transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr)
{
  PlMatrix_float M(4,4);
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
      M(i,j) = rot(i,j);
    M(i,3) = tr(i);
  }
  M(3,0)=M(3,1)=M(3,2)=0;
  M(3,3)=1;

  MatrixRTf T(M);
  //  std::cout<<T<<"\n";
  //  nurbs_.writeVRML97("A.wrl");
  //  std::cout<<nurbs_.ctrlPnts()<<"\n\n";
  nurbs_.transform(T);
  //  nurbs_.writeVRML97("B.wrl");
  //  std::cout<<nurbs_.ctrlPnts()<<"\n\n";
}

bool SurfaceNurbs::fitsCurvature(const Surface &o, const float thr) const
{
  ROS_ASSERT(getSurfaceType()==getSurfaceType());

  return false;//TODO: return param_.col(2).cross(((SurfaceNurbs*)&o)->param_.col(2)).squaredNorm() < thr*thr;
}

size_t ___CTR___ = 0;

int SurfaceNurbs::myprojectOn(const SURFACE &s, const Eigen::Vector3f &p, float &u, float &v, const int maxIt) const
{
  PlMatrix_Point3Df ders;
  Eigen::Vector3f dx, dy, dz, t, d2;
  Eigen::Vector3f v3, q;
  Eigen::Vector2f v2;
  Eigen::Matrix3f M = Eigen::Matrix3f::Identity();
  //  Eigen::Matrix<float,3,2> H, G;
  float du, dv;
  int it = 0;

  while(it < maxIt)
  {
    //project p to plane -> q
    s.deriveAt(u,v,2,ders) ;
    dy(0) = ders(0,1).x();
    dy(1) = ders(0,1).y();
    dy(2) = ders(0,1).z();
    dx(0) = ders(1,0).x();
    dx(1) = ders(1,0).y();
    dx(2) = ders(1,0).z();
    d2(0) = ders(1,1).x();
    d2(1) = ders(1,1).y();
    d2(2) = ders(1,1).z();
    t(0) = ders(0,0).x();
    t(1) = ders(0,0).y();
    t(2) = ders(0,0).z();

    dz = dx.cross(dy);
    if(dz.squaredNorm()) dz.normalize();

    q = dz.dot(t-p)*dz + p - t;

    //solve q = dx*du + dy*dv
    M.col(0) = dx;
    M.col(1) = dy;
    //    M.col(2) = dz;

    v3 = M.inverse()*q;//M.llt().solve(q);

    //    const float k = (dx(0)*v3(0)*v3(0) + dx(1)*v3(1)*v3(0) + dy(0)*v3(0)*v3(1) + dy(1)*v3(1)*v3(1)) /
    //        (d2(0)*v3(0)*v3(0) + d2(1)*v3(1)*v3(0) + dz(0)*v3(0)*v3(1) + dz(1)*v3(1)*v3(1));
    //    cs = v3(0)*dx + v3(1)*dy;
    //    const float dt = (c.cros(q-p))/(k*std::pow(cs.norm(),3));

    du = v3(0);
    dv = v3(1);

    //    std::cout<<ders<<"\n";

    //    dx/=dx.norm();
    //    dy/=dy.norm();

    //    M.col(0) = dx;
    //    M.col(1) = dy;
    //    M.col(2) = dx.cross(dy);
    //
    //    t = (p-t);
    //    //t = M.transpose()*t;
    //    du = t(0);
    //    dv = t(1);

    //        v2(0) = u;
    //        v2(1) = v;
    //        t = _project2world(v2);
    //
    //        t+=du*dx;
    //        t+=dv*dy;


    //    std::cout<<"SN M\n"<<dx.dot(dy)<<"\n";
    //        std::cout<<"SN M\n"<<M.transpose()<<"\n";
    //        std::cout<<"SN M\n"<<M.transpose()*t<<"\n";
    //    std::cout<<"SN dot\n"<<q.dot(dz)<<"\n";
    //    std::cout<<"SN v3\n"<<v3<<"\n";
    //    std::cout<<"SN t\n"<<t<<"\n";
    //    std::cout<<"SN p\n"<<p<<"\n";
    //    std::cout<<"SN q\n"<<q<<"\n";
    //    std::cout<<"SN dx\n"<<dx<<"\n";
    //    std::cout<<"SN dy\n"<<dy<<"\n";
    //    std::cout<<"SN delta "<<du<<" "<<dv<<"\n";

    //    ROS_ASSERT(std::abs(q.dot(dz))<0.1f);

    u+=du;
    v+=dv;

    if(std::abs(du)+std::abs(dv)<0.00001f) {
      break;
    }

    ++it;
  }
  return 0;
}

/// find nearest point to manifold
Eigen::Vector2f SurfaceNurbs::nextPoint(const Eigen::Vector3f &v) const
{
  Eigen::Vector2f r = _nextPoint(v);
  r(0) = r(0)/c_x_m_ - c_x_o_;
  r(1) = r(1)/c_y_m_ - c_y_o_;
  return r;
}

/// find nearest point to manifold
Eigen::Vector2f SurfaceNurbs::_nextPoint(const Eigen::Vector3f &v) const
{
  ++___CTR___;
#if 0
  Eigen::VectorXf r(2);
  r(0) = 0.5f;
  r(1) = 0.5f;

  std::cout<<"___________START_____________\n";

  Functor f(nurbs_);
  f._pt_.x() = v(0);
  f._pt_.y() = v(1);
  f._pt_.z() = v(2);
  Eigen::LevenbergMarquardt<Functor, float> lm(f);
  lm.parameters.maxfev = 50; //not to often (performance)
  lm.minimize(r);

  return r;

#else
  PlPoint3Df p(v(0),v(1),v(2));
  float Ub, Vb, dist;
  int res2;

  for(int i=0; i<nurbs_.ctrlPnts().rows()-1; i++)
    for(int j=0; j<nurbs_.ctrlPnts().cols()-1; j++)
    {
#if 0
      float U=(i+0.5f)/(nurbs_.ctrlPnts().rows()-1),
          V=(j+0.5f)/(nurbs_.ctrlPnts().cols()-1);
      int res = myprojectOn(nurbs_, v, U, V, 4);
      //      float U1=vals[i],V1=vals[j];
      //      nurbs_.projectOn(p, U1, V1, 50, -100.f, 100.f, -100.f, 100.f);
      //        std::cout<<"SN GT "<<U1<<" "<<V1<<"\n";
      //        std::cout<<"SN NA "<<U<<" "<<V<<"\n";

      Eigen::Vector2f r;
      r(0) = U;
      r(1) = V;
      float d = (_project2world(r)-v).squaredNorm();
#else
      float U=(i+0.5f)/(nurbs_.ctrlPnts().rows()-1),
          V=(j+0.5f)/(nurbs_.ctrlPnts().cols()-1);
      int res = 1;
      Eigen::Vector3f vCP;
      vCP(0) = (nurbs_.ctrlPnts()(i,j).x()+nurbs_.ctrlPnts()(i+1,j).x()+nurbs_.ctrlPnts()(i,j+1).x()+nurbs_.ctrlPnts()(i+1,j+1).x())/4;
      vCP(1) = (nurbs_.ctrlPnts()(i,j).y()+nurbs_.ctrlPnts()(i+1,j).y()+nurbs_.ctrlPnts()(i,j+1).y()+nurbs_.ctrlPnts()(i+1,j+1).y())/4;
      vCP(2) = (nurbs_.ctrlPnts()(i,j).z()+nurbs_.ctrlPnts()(i+1,j).z()+nurbs_.ctrlPnts()(i,j+1).z()+nurbs_.ctrlPnts()(i+1,j+1).z())/4;
      float d = (v-vCP).squaredNorm();
#endif

      //      std::cout<<d<<" "<<U<<" "<<V<<"\n";

      if((!j&&!i) || d<dist)
      {
        Ub = U;
        Vb = V;
        dist = d;
        res2=res;
      }
    }

  //  float U=0.25f, V=0.75f;
  //  myprojectOn(nurbs_, v, U, V, 3);

  //  std::cout<<"SN GT "<<Ub<<" "<<Vb<<"\n";
  //  std::cout<<"SN NA "<<U<<" "<<V<<"\n";

  //  Ub=0.5f;
  //  Vb=0.5f;
  //  nurbs_.minDist2(p, Ub, Vb, 0.0001f, 2.f, 5, 50, -100.f, 100.f, -100.f, 100.f);//nurbs_.projectOn(p, U, V, 100, -10000.f, 10000.f, -10000.f, 10000.f);

  //  std::cout<<"p1: "<<p<<"\n";
  //  std::cout<<"res: "<<res2<<"\n";
  //  std::cout<<"guess x: "<<plane_x_.dot(v)<<"\n";
  //  std::cout<<"guess y: "<<plane_y_.dot(v)<<"\n";

  myprojectOn(nurbs_, v, Ub, Vb, 30);

  Eigen::Vector2f r;
  r(0) = Ub;
  r(1) = Vb;
  return r;
#endif
}

/// merge parameters
float SurfaceNurbs::merge(const Surface &o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) {
  ROS_ASSERT(getSurfaceType()==o.getSurfaceType());
  return _merge(*(const SurfaceNurbs*)&o, this_w, o_w, wind_t, wind_o);
}

#define _INSERT(Xs,X) \
if(!Xs.size()||Xs.back()<=X) \
Xs.push_back(X); \
else \
  for(int i=Xs.size()-1; i>=0; i--) \
    if(i<1||Xs[i-1]<=X) \
      {Xs.insert(Xs.begin()+i,X);break;}

float SurfaceNurbs::_merge(SurfaceNurbs o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) {
  const int SAMPLING = 1;

  float err=0.f;
#if 1

  // 1. find overlapping points within [0,1] (SVD)

  Eigen::Vector2f wind_t1, wind_t2;
  Eigen::Vector2f wind_o1, wind_o2;

  wind_t1(0) = wind_t.min_x;
  wind_t1(1) = wind_t.min_y;
  wind_t2(0) = wind_t.max_x;
  wind_t2(1) = wind_t.max_y;

  wind_o1(0) = wind_o.min_x;
  wind_o1(1) = wind_o.min_y;
  wind_o2(0) = wind_o.max_x;
  wind_o2(1) = wind_o.max_y;

  wind_t1 = correct(wind_t1);
  wind_t2 = correct(wind_t2);
  wind_o1 = o.correct(wind_o1);
  wind_o2 = o.correct(wind_o2);

#ifdef USE_NURBS_TRANSFORM_
  DOF6::TFLinkvf magic_box;
#endif

  std::vector<float> us1, vs1;
  std::vector<float> us2, vs2;

#ifdef USE_NURBS_TRANSFORM_
  for(int i=0; i<nurbs_.ctrlPnts().rows(); i++) {
    for(int j=0; j<nurbs_.ctrlPnts().cols(); j++) {
      Eigen::Vector2f v2;
      v2(0)=i/(float)(nurbs_.ctrlPnts().rows()-1) * (wind_t2(0)-wind_t1(0)) + wind_t1(0);
      v2(1)=j/(float)(nurbs_.ctrlPnts().cols()-1) * (wind_t2(1)-wind_t1(1)) + wind_t1(1);
      Eigen::Vector3f v = _project2world(v2);

      Eigen::Vector2f np = o._nextPoint(v);
      PlPoint2Df NP;
      NP.x()=np(0);
      NP.y()=np(1);

//      std::cout<<NP.x()<<" "<<NP.y()<<"\n";
      float _u2 = (NP.x()-wind_o1(0))/(wind_o2(0)-wind_o1(0));
      float _v2 = (NP.y()-wind_o1(1))/(wind_o2(1)-wind_o1(1));

      if(_u2<=1.f && _u2>=0.f && _v2<=1.f && _v2>=0.f)
      {
        magic_box(
            DOF6::TFLinkvf::TFLinkObj(v, false,false),
            DOF6::TFLinkvf::TFLinkObj(o._project2world(np),false,false));

        //float u = i/(float)nurbs_.degreeU();
        //float v = j/(float)nurbs_.degreeV();
      }

    }
  }
#endif
  us2.resize(o.nurbs_.ctrlPnts().rows(),0);
  vs2.resize(o.nurbs_.ctrlPnts().cols(),0);

  float miU=10, miV=10, maU=-10, maV=-10;
  for(int i=0; i<o.nurbs_.ctrlPnts().rows(); i++) {
    for(int j=0; j<o.nurbs_.ctrlPnts().cols(); j++) {
      Eigen::Vector2f v2;
      v2(0)=i/(float)(o.nurbs_.ctrlPnts().rows()-1) * (wind_o2(0)-wind_o1(0)) + wind_o1(0);
      v2(1)=j/(float)(o.nurbs_.ctrlPnts().cols()-1) * (wind_o2(1)-wind_o1(1)) + wind_o1(1);
      Eigen::Vector3f v = o._project2world(v2);

      Eigen::Vector2f np = _nextPoint(v);
      PlPoint2Df NP;
      NP.x()=np(0);
      NP.y()=np(1);

//      std::cerr<<"err1 "<<(_project2world(np)-v).norm()<<"\n";
//      std::cerr<<"err2 "<<(o._project2world(o._nextPoint(_project2world(np)))-v).norm()<<"\n";

      us2[i] += np(0);
      vs2[j] += np(1);

      float _u1 = (NP.x()-wind_t1(0))/(wind_t2(0)-wind_t1(0));
      float _v1 = (NP.y()-wind_t1(1))/(wind_t2(1)-wind_t1(1));

      if(_u1<=1.f && _u1>=0.f && _v1<=1.f && _v1>=0.f)
      {

#ifdef USE_NURBS_TRANSFORM_
        magic_box(
            DOF6::TFLinkvf::TFLinkObj(_project2world(np),false,false),
            DOF6::TFLinkvf::TFLinkObj(v, false,false));
#endif

      }
      else if( o._normalAt(v2).dot(_normalAt(np)) < .97f /*(v-_project2world(np)).squaredNorm()>0.02f*std::min(v.squaredNorm(),_project2world(np).squaredNorm())*/ ){
        miU = std::min(miU, NP.x());
        miV = std::min(miV, NP.y());
        maU = std::max(maU, NP.x());
        maV = std::max(maV, NP.y());
      }

    }
  }
  
  for(int j=0; j<nurbs_.ctrlPnts().rows(); j++) {
  float u = j/(float)(nurbs_.ctrlPnts().rows()-1) * (wind_t2(0)-wind_t1(0)) + wind_t1(0);
  _INSERT(us1,u);
  }
  for(int i=0; i<nurbs_.ctrlPnts().cols(); i++) {
  float v = i/(float)(nurbs_.ctrlPnts().cols()-1) * (wind_t2(1)-wind_t1(1)) + wind_t1(1);
  _INSERT(vs1,v);
  }

  for(int j=0; j<us2.size(); j++) {
    float u = us2[j]/o.nurbs_.ctrlPnts().cols();
    //std::cerr<<"u "<<u<<"\n";
    _INSERT(us1,u);
  }
  for(int i=0; i<vs2.size(); i++) {
    float v = vs2[i]/o.nurbs_.ctrlPnts().rows();
    //std::cerr<<"v "<<v<<"\n";
    _INSERT(vs1,v);
  }

#if DOME
  sorteed
  for(int j=0; j<o.nurbs_.ctrlPnts().rows(); j++) {
  float u = j/(float)(o.nurbs_.ctrlPnts().rows()-1) * (wind_o2(0)-wind_o1(0)) + wind_o1(0);
  _INSERT(us1,u);
  }
  for(int i=0; i<o.nurbs_.ctrlPnts().cols(); i++) {
  float v = i/(float)(o.nurbs_.ctrlPnts().cols()-1) * (wind_o2(1)-wind_o1(1)) + wind_o1(1);
  _INSERT(vs1,v);
  }
#endif

  int nU = std::max(nurbs_.ctrlPnts().rows(),o.nurbs_.ctrlPnts().rows());
  int nV = std::max(nurbs_.ctrlPnts().cols(),o.nurbs_.ctrlPnts().cols());

  const float offU = 0.03f*nU;
  const float offV = 0.03f*nV;

#ifdef USE_NURBS_TRANSFORM_
  // 2. find and apply transformation to register both surfaces
  magic_box.finish();

  {
    Eigen::AngleAxisf aa;
    aa.fromRotationMatrix(magic_box.getRotation());
    Eigen::AngleAxisf aa2(0.5f*o_w/(this_w+o_w)*aa.angle(),aa.axis());
    transform(aa2.toRotationMatrix(), 0.5f*o_w/(this_w+o_w)*magic_box.getTranslation());
  }

  magic_box = magic_box.transpose();

  {
    Eigen::AngleAxisf aa;
    aa.fromRotationMatrix(magic_box.getRotation());
    Eigen::AngleAxisf aa2(0.5f*this_w/(this_w+o_w)*aa.angle(),aa.axis());

    //o.transform(magic_box.getRotation(),magic_box.getTranslation());
    o.transform(aa2.toRotationMatrix(), 0.5f*this_w/(this_w+o_w)*magic_box.getTranslation());
  }

  std::cout<<"OPTIMIZED MERGE\n"<<magic_box<<"\n";
#endif

  //insert new knots

//  if(miU<-offU) {nU++; us1.insert(us1.begin(),miU);}
//  if(miV<-offV) {nV++; vs1.insert(vs1.begin(),miV);}
//  if(maU>1.f+offU) {nU++; us1.push_back(maU);}
//  if(maV>1.f+offV) {nV++; vs1.push_back(maV);}

#if 1
  {
    float _miU, _maU, _miV, _maV;
    _miU = (miU-wind_t1(0))/(wind_t2(0)-wind_t1(0));
    _miV = (miV-wind_t1(1))/(wind_t2(1)-wind_t1(1));
    _maU = (maU-wind_t1(0))/(wind_t2(0)-wind_t1(0));
    _maV = (maV-wind_t1(1))/(wind_t2(1)-wind_t1(1));
  if(
      (_miU>1&&_maU>1) ||
      (_miV>1&&_maV>1) ||
      (_miU<0&&_maU<0) ||
      (_miV<0&&_maV<0)<0
      )
    return 100000.f;

    if(_miU<-offU)
    {
      nU++;
      //_INSERT(us1,miU);
    }
    if(_miV<-offV)
    {
      nV++;
      //_INSERT(vs1,miV);
    }
    if(_maU>1.f+offU)
    {
      nU++;
      //_INSERT(us1,maU);
    }
    if(_maV>1.f+offV)
    {
      nV++;
      //_INSERT(vs1,maV);
    }
  }
#endif

#if 0
    for(size_t i=0; i<(us1.size()-1)*SAMPLING+1; i++) {
      if(!(i%SAMPLING)) {
        std::cerr<<"u "<< us1[i/SAMPLING]<<"\n";
      }
      else {
        std::cerr<<"u "<< (us1[i/SAMPLING]*(SAMPLING-i%SAMPLING)+us1[i/SAMPLING+1]*(i%SAMPLING))/SAMPLING<<"\n";
      }
    }
    for(size_t j=0; j<(vs1.size()-1)*SAMPLING+1; j++) {
      if(!(j%SAMPLING)) {
        std::cerr<<"v "<<  vs1[j/SAMPLING]<<"\n";
      }
      else {
        std::cerr<<"v "<<  (vs1[j/SAMPLING]*(SAMPLING-j%SAMPLING)+vs1[j/SAMPLING+1]*(j%SAMPLING))/SAMPLING<<"\n";
      }
    }
#endif

//    if(miU<-offU) {nU++;}
//    if(miV<-offV) {nV++;}
//    if(maU>1.f+offU) {nU++;}
//    if(maV>1.f+offV) {nV++;}

//  ROS_ASSERT(us1.size()==us2.size());
//  ROS_ASSERT(vs1.size()==vs2.size());

  // 3. create point grid with (m+j)x(n+k) points (mxn from surf. 1 and jxk from surf. 2)

  Matrix_Point3Df Pts((us1.size()-1)*SAMPLING+1,(vs1.size()-1)*SAMPLING+1);

  float n=0;
  bool inv=false;
  for(size_t i=0; i<(us1.size()-1)*SAMPLING+1; i++) {
    for(size_t j=0; j<(vs1.size()-1)*SAMPLING+1; j++) {

      Eigen::Vector2f np1,np2;
      Eigen::Vector3f v, v2;

      if(!(i%SAMPLING))
        np1(0) = us1[i/SAMPLING];
      else
        np1(0) = (us1[i/SAMPLING]*(SAMPLING-i%SAMPLING)+us1[i/SAMPLING+1]*(i%SAMPLING))/SAMPLING;

      if(!(j%SAMPLING))
        np1(1) = vs1[j/SAMPLING];
      else
        np1(1) = (vs1[j/SAMPLING]*(SAMPLING-j%SAMPLING)+vs1[j/SAMPLING+1]*(j%SAMPLING))/SAMPLING;

      v=_project2world(np1);
      np2 = o._nextPoint(v);
      v2= o._project2world(np2);

      float _u1 = (np1(0)-wind_t1(0))/(wind_t2(0)-wind_t1(0));
      float _u2 = (np2(0)-wind_o1(0))/(wind_o2(0)-wind_o1(0));
      float _v1 = (np1(1)-wind_t1(1))/(wind_t2(1)-wind_t1(1));
      float _v2 = (np2(1)-wind_o1(1))/(wind_o2(1)-wind_o1(1));

//      float _u1 = us1[i];
//      float _u2 = np2(0);
//      float _v1 = vs1[i];
//      float _v2 = np2(1);

      bool b1In = _u1>=0.f && _u1<=1.f && _v1>=0.f && _v1<=1.f;
      bool b2In = _u2>=0.f && _u2<=1.f && _v2>=0.f && _v2<=1.f;

//      float w1 = b1In ? 1.f : (0.25f/((0.5f-_u1)*(0.5f-_u1) + (0.5f-_v1)*(0.5f-_v1))),
//            w2 = b2In ? 1.f : (0.25f/((0.5f-_u2)*(0.5f-_u2) + (0.5f-_v2)*(0.5f-_v2)));
      float w1 = (0.25f/((0.5f-_u1)*(0.5f-_u1) + (0.5f-_v1)*(0.5f-_v1)+0.25f)),
            w2 = (0.25f/((0.5f-_u2)*(0.5f-_u2) + (0.5f-_v2)*(0.5f-_v2)+0.25f));

      err += (v-v2).norm()*std::min(w1,w2);
      n   += std::min(w1,w2);

      //Debug::Interface::get().addArrow(v,v2,200,255,110);

      //if(!(b1In^b2In))
        v = (w1*this_w*v + w2*o_w*v2)/(w1*this_w+w2*o_w);
      //else if(b2In)
      //  v = v2;
        //std::cerr<<"w "<<w1<<" "<<w2<<"\n";
        //ROS_ASSERT(w1+w2>0.1f);

        /*
      //merge if it is both inside/both outside
      if(!(b1In^b2In))
      {
        v2= o._project2world(np2);
        err = std::max(err, (v-v2).norm());
        //if((v-v2).norm()<0.02f*std::min(v.squaredNorm(),v2.squaredNorm()))
          v = (this_w*v + o_w*v2)/(this_w+o_w);
      }
      else if(b2In)
      {
        v2= o._project2world(np2);
        //if((v-v2).norm()<0.05f*std::min(v.squaredNorm(),v2.squaredNorm()))
          v = v2;
      }*/

      if(!pcl_isfinite(v.sum())) {
        inv=true;
        break;
      }

      Pts(i,j).x() = v(0);
      Pts(i,j).y() = v(1);
      Pts(i,j).z() = v(2);
    }
  }

  err /= n;

  if(inv) {
    ROS_WARN("invalid");
    return 10000.f;
  }

  Eigen::Vector2f t2;
  t2(0) = 0.;
  t2(1) = 0.f;
  Eigen::Vector3f test = _project2world(t2);

//  ROS_INFO("degree %d %d",Pts.cols(),Pts.rows());
//  ROS_INFO("degree before %d %d",nurbs_.ctrlPnts().cols(),nurbs_.ctrlPnts().rows());
  //nurbs_.globalInterp(Pts,2,2);
  nurbs_.leastSquares(Pts,2,2, nU, nV);
  //PLib::globalSurfApprox(Pts, 2, 2, nurbs_, 0.05);
//  ROS_INFO("degree after  %d %d",nurbs_.ctrlPnts().cols(),nurbs_.ctrlPnts().rows());
//  ROS_INFO("nurbs error  %f",err);

//  std::cout<<"2D\n"<<t2<<"\n";
//  t2 = _nextPoint(test);
//  std::cout<<"2D\n"<<t2<<"\n";
//  std::cout<<"3D1\n"<<test<<"\n";
//  std::cout<<"3D2\n"<<_project2world(t2)<<"\n";
//  std::cout<<"3D3\n"<<Pts(0,0)<<"\n";

#else

  Matrix_Point3Df Pts(3,3);
  for(int i=0;i<3;++i){
    for(int j=0;j<3;++j){
      const float x = (i*0.5f);
      const float y = (j*0.5f);
      Pts(i,j) = nurbs_.pointAt(x,y);
    }
  }

  for(int i=0;i<3;++i){
    for(int j=0;j<3;++j){
      float x = (i*0.5f);
      float y = (j*0.5f);

      o.nurbs_.minDist2(Pts(i,j), x, y, 0.00001f, 100.f, 20, 100, -10000.f, 10000.f, -10000.f, 10000.f); //perhaps check distance?
      Pts(i,j) = (this_w*Pts(i,j) + o_w*o.nurbs_.pointAt(x,y))/(this_w+o_w);


      //      Eigen::Vector3f v;
      //      v(0)=Pts(i,j).x();
      //      v(1)=Pts(i,j).y();
      //      v(2)=Pts(i,j).z();
      //      Eigen::Vector2f np = o.nextPoint(v);
      //      //o.nurbs_.minDist2(Pts(i,j), x, y, 0.00001f, 100.f, 20, 100, -10000.f, 10000.f, -10000.f, 10000.f); //perhaps check distance?
      //      v = (this_w*v + o_w*o.project2world(np))/(this_w+o_w);
      //      Pts(i,j).x() = v(0);
      //      Pts(i,j).y() = v(1);
      //      Pts(i,j).z() = v(2);
    }
  }

  nurbs_.globalInterp(Pts,2,2);
#endif
  return err;
}

float SurfaceNurbs::_merge2(SurfaceNurbs o, const float this_w, const float o_w, const SWINDOW &wind_t, const SWINDOW &wind_o) {
  const int SAMPLING = 10;
#if 0
  //generate grid
  PT_GRID ga(nurbs_,SAMPLING), gb(o.nurbs_,SAMPLING);

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::PointXYZ pt;
  pcl::PCA<pcl::PointXYZ> pca;

  for(int x=0; x<ga.w; x++) {
    for(int y=0; y<ga.h; y++) {
      Eigen::Vector2f v;
      v(0) = x/(float)(ga.w-1);
      v(1) = y/(float)(ga.h-1);
      Eigen::Vector3f p = _project2world(v);
      ga(x,y) = p;
      pt.x=p(0);pt.y=p(1);pt.z=p(2);
      pc.push_back(pt);
    }
  }

  for(int x=0; x<gb.w; x++) {
    for(int y=0; y<gb.h; y++) {
      Eigen::Vector2f v;
      v(0) = x/(float)(gb.w-1);
      v(1) = y/(float)(gb.h-1);
      Eigen::Vector3f p = o._project2world(v);
      gb(x,y) = p;
      pt.x=p(0);pt.y=p(1);pt.z=p(2);
      pc.push_back(pt);
    }
  }

  pca.compute(pc);
  int c1=0;
  if(pca.getEigenValues()(c1)<pca.getEigenValues()(1)) c1=1;
  if(pca.getEigenValues()(c1)<pca.getEigenValues()(2)) c1=2;

  int found, pos_x, pos_y;
  Eigen::Vector3f n,m;
  n=pca.getEigenVectors().col(c1);

  std::vector< std::vector<Eigen::Vector3f> > lines1;
  m=pca.getMean();
  pos_y=pos_x=0;
  do {
    lines1.push_back(std::vector<Eigen::Vector3f>());
    found=0;

    for(int x=0; x<ga.w-1; x++) {
      float f=n.dot(p3-ga(x,pos_y))/n.dot(ga(x+1,pos_y)-ga(x,pos_y));
      if(f>=0&&f<1) {
        break;df
      }
    }

    m+=n/SAMPLING;
  } while(found>0);
#endif
  float err=0.f;
  return err;
}
