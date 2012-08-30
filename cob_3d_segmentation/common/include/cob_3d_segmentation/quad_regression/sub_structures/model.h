/*
 * model.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef MODEL_H_
#define MODEL_H_

#define USE_SVD_

/**
 * Model contains data for regression calculations and parameters
 */
struct Model {
  Param::Vector6f p;
  Param param;

  Model() {}

  Model(const Param &p) {*this+=p;}

  /**
   * check in z-axis with theshold (thr)
   */
  inline bool checkZ(const Param &para, const float thr) const {
    float d=std::abs((para.model_.row(0)*p)(0)-para.z_(0));
    return d<thr*para.model_(0,0);
  }

  /**
   * check wether it could be linear, and convert it
   */
  bool isLinearAndTo()
  {
    //Variance for both axis (spare small objects)
    const float Vx = (param.model_(1,1)-param.model_(0,1)*param.model_(0,1)/param.model_(0,0))/param.model_(0,0);
    const float Vy = (param.model_(3,3)-param.model_(0,3)*param.model_(0,3)/param.model_(0,0))/param.model_(0,0);

    if(Vx<0.001f && Vy<0.001f)
    {
//      ROS_INFO("np1 %f %f", Vx, Vy);
      return false;
    }

    Model temp=*this;
    temp.getLinear2();

    //const float error1 = (model_.param.z_ - model_.param.model_*model_.p).squaredNorm();
    const float error2 = (temp.param.z_ - temp.param.model_*temp.p).norm();
    const float d=temp.param.z_(0)/temp.param.model_(0,0);

    if(//std::abs(param.model_.determinant())*std::abs(param.z_(0))>0.0001f*param.model_(0,0)*param.model_(0,0) &&
        (error2 > (temp.param.model_(0,0)*temp.param.model_(0,0))/(1<<13)*d*d || (std::abs(p(2))+std::abs(p(4))+std::abs(p(5)))/(std::abs(p(1))+std::abs(p(3)))>1.f))
    {
//      ROS_INFO("np2 %f",std::abs(param.model_.determinant()/param.model_(0,0)));
      return false;
    }

    *this = temp;

//    ROS_INFO("plane");

    return true;
  }

  /**
   * get maximum of gradient
   */
  inline float get_max_gradient(const Param &para) const {
    return
        std::abs(2.f*para.model_(0,1)/para.model_(0,0)*p(2)+p(1)+para.model_(0,3)/para.model_(0,0)*p(5))
    + std::abs(2.f*para.model_(0,3)/para.model_(0,0)*p(4)+p(3)+para.model_(0,1)/para.model_(0,0)*p(5));
  }

  /**
   * approximation to nearest point from view point to check against
   */
  inline bool check_tangent(const Param &para, const float thr) const {
    const float t=(para.z_(0)-(para.model_.row(0)*p)(0))/para.model_(0,0);
    Eigen::Vector3f r;
    r(0)=2.f*para.model_(0,1)/para.model_(0,0)*p(2)+p(1)+para.model_(0,3)/para.model_(0,0)*p(5);
    r(1)=2.f*para.model_(0,3)/para.model_(0,0)*p(4)+p(3)+para.model_(0,1)/para.model_(0,0)*p(5);
    r(2)=1.f;

    return t*t<thr*thr*r.squaredNorm();
  }

  /**
   * z-value at
   */
  float model(const Param &para) const {
    //return (p(2)*para.model_(0,2)+p(4)*para.model_(0,4) + p(1)*para.model_(0,1)+p(3)*para.model_(0,3))/para.model_(0,0) + p(0);
    return (para.model_.row(0)*p)(0)/para.model_(0,0);
  }

  /**
   * z-value at
   */
  float model(const float x, const float y) const {
    return p(0)+p(1)*x+p(2)*x*x+p(3)*y+p(4)*y*y+p(5)*x*y;
  }

  float getCorrelation() const { //TODO: not correct
    return (
        param.model_(1,3)-param.model_(0,1)*param.model_(0,3)/param.model_(0,0)
    )
    /sqrtf(
        (param.model_(1,1)-param.model_(0,1)*param.model_(0,1)/param.model_(0,0))*
        (param.model_(3,3)-param.model_(0,3)*param.model_(0,3)/param.model_(0,0))
    );
  }

  /**
   * do regression for 6 parameters
   * TODO: if it's too steep (thr) make it linear
   */
  void get() {
    if(param.model_(0,0)==0) {
      p(0)=p(1)=p(2)=p(3)=p(4)=p(5)=0.f;
    }
//    else if(param.model_(0,0)>1000 && std::abs(param.model_.determinant()/param.model_(0,0))<0.00001f)
//    {
//      getLinear2();
//    }
    else {

#ifdef USE_SVD_
      bool bLDLT=false;
      // compute the SVD:
      Eigen::JacobiSVD<Param::Matrix6f> svd (param.model_, Eigen::ComputeFullU | Eigen::ComputeFullV);
      const Param::Matrix6f& u = svd.matrixU(),
          & v = svd.matrixV();
      Param::Vector6f s = svd.singularValues();

      const Param::Vector6f d = u.transpose()*param.z_;

      //int index_map[6]={0,1,3,2,4,5};
      // determine the effective rank r of A using singular values
      int r = 0;
      Param::Vector6f t = Param::Vector6f::Zero();
      while( r < 6 && s(r) >= 0.0001f )
      {
        t(r) = d(r) / s(r);
        r++;
      }

      p = v * t;

#else
//      std::cout<<"p\n"<<p<<"\n";
//      std::cout<<"t\n"<<t<<"\n";
//      std::cout<<"d\n"<<d<<"\n";
//      std::cout<<"s\n"<<s<<"\n";
//      std::cout<<"v\n"<<v<<"\n";

      bool bLDLT=param.z_(0)/param.model_(0)<1.2f;

      if(bLDLT)
        p=param.model_.ldlt().solve(param.z_);
      else
        p=param.model_.fullPivLu().solve(param.z_);
#endif

      if(!pcl_isfinite(p(1))) {
        if(bLDLT)
          p=param.model_.fullPivLu().solve(param.z_);
        else
          p=param.model_.ldlt().solve(param.z_);

        if(!pcl_isfinite(p(1))) {
          ROS_ERROR("mist2");
          std::cerr<<param.model_<<"\n";
          std::cerr<<param.z_<<"\n";
          getchar();
        }
      }

    }
  }

  Eigen::Matrix3f getLinearMatrix() const {
    Eigen::Matrix3f M;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        M(i,j)=param.model_(i>1?3:i, j>1?3:j);
    return M;
  }

  Eigen::Vector3f getLinear() const {
    Eigen::Vector3f r;
    r.fill(0);
    if(param.model_(0,0)!=0) {
      Eigen::Matrix3f M;
      Eigen::Vector3f z;
      z(0)=param.z_(0);
      z(1)=param.z_(1);
      z(2)=param.z_(3);
      for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
          M(i,j)=param.model_(i>1?3:i, j>1?3:j);
      //_p=M.inverse()*z;
      r=M.fullPivLu().solve(z);

      //r=M.inverse()*z;
    }

    return r;
  }

  void getLinear2()
  {
    Eigen::Vector3f r=getLinear();
    p(0)=r(0);
    p(1)=r(1);
    p(2)=0;
    p(3)=r(2);
    p(4)=0;
    p(5)=0;
  }

  Eigen::Vector3f getLinearNormal() {
    Eigen::Vector3f n,t=getLinear();
    n(2)=1;
    n(0)=t(1);
    n(1)=t(2);
    n.normalize();
    return n;
  }

  Eigen::Vector3f getNormal(const float x, const float y) const {
    Eigen::Vector3f n;
    n(2)=1;
    n(0)=-(p(1)+2*x*p(2)+p(5)*y);
    n(1)=-(p(3)+2*x*p(4)+p(5)*x);
    n.normalize();
    return n;
  }

  void operator+=(const Model &m) {
    param+=m.param;
  }
  void operator+=(const Param &m) {
    param+=m;
  }

  float intersect(const float mx, const float my) const {
    float a=mx*mx*p(2) + my*my*p(4) + mx*my*p(5);
    float b=mx*p(1) + my*p(3) - 1;

    if(std::abs(a)<0.00001f) {
      return -p(0)/b;
    }

    float t,z2;
    //if(t<0.f && t>-0.1)
      //  return (-b)/(2*a);
    t=b*b-4*a*p(0);
    z2=(-b-sqrtf(t))/(2*a);
    /*if(!pcl_isfinite(z2)) {
          a*=0.95f;
          t=b*b-4*a*p(0);
          z2=(-b-sqrtf(t))/(2*a);
        }*/

#if DEBUG_LEVEL >70
    if(!pcl_isfinite(z2)) {
      std::cerr<<"t: "<<t<<"\n";
      std::cerr<<"a: "<<a<<"\n";
    }
#endif
    /*
        float z1=(-b+sqrtf(b*b-4*a*p(0)))/(2*a);

        std::cout<<"a: "<<a<<" b: "<<b<<" c: "<<p(0)<<"\n";
        std::cout<<"mx: "<<mx<<" my: "<<my<<"\n";
        std::cout<<"z: "<<z2<<" z: "<<z1<<"\n";
        std::cout<<"m: "<<<<"\n";*/

    return z2;
  }
};


#endif /* MODEL_H_ */
