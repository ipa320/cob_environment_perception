/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
/*
 * model.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef MODEL_H_
#define MODEL_H_

namespace intern {

  /// ray cast surface with ray $z = (mx my 1)$ --> null points of polynomial equation (=roots)
  template<int Degree2>
  inline float _intersect(const float mx, const float my, const typename Param<Degree2>::VectorU &p) {
    Eigen::Matrix<float, Degree2+1, 1> _p;

    int n = 0;
    _p.fill(0);
    _p(1) = -1.f;
    for(int i=0; i<=Degree2; i++)
      for(int j=0; j<=i; j++) {
        _p(i) += p(i*(i+1)/2+j)*std::pow(mx,j)*std::pow(my,i-j);
        if(std::abs(_p(i))>0.000001f) n=i;
      }

    std::vector<double> realRoots;
    if(n==Degree2) {
      Eigen::PolynomialSolver<float, Degree2> ps(_p);
      ps.realRoots( realRoots );
    }
    else if(n==0)
      return _p(0);
    else if(n==1)
      return -_p(0)/_p(1);
    else {
      Eigen::PolynomialSolver<float, Eigen::Dynamic> ps(_p.head(n+1));    //reduce degree to n
      ps.realRoots( realRoots );
    }

    for(size_t i=0; i<realRoots.size(); i++)
      if(realRoots[i]>0) return realRoots[i];

    return 0.f;
  }

  /// ray cast surface with ray $z = (mx my 1)$ --> null points of polynomial equation (=roots)
  template<>
  inline float _intersect<1>(const float mx, const float my, const typename Param<1>::VectorU &p) {
    return -p(0)/(my*p(1)+mx*p(2)-1);
  }
}


/**
 * Model contains data for regression calculations and parameters
 */
template <int Degree>
struct Model {
  typedef Eigen::Matrix<float, 2*Degree+1, 1>    VectorU1D;
  typedef Eigen::Matrix<float, 2*Degree+1, 2*Degree+1>    MatrixU1D;
  
  Param<Degree> param;
  typename Param<Degree>::VectorU p;

  Model() {p.fill(0.f);}
  Model(const Param<Degree> &pa) {p.fill(0.f); *this+=pa;}
  
  /// transform parameters from 2D input f(x,y) to 1D input f(t) (from origin 0/0)
  inline VectorU1D transformation_1D(const Eigen::Vector2f &dir, const Eigen::Vector2f &off, const Eigen::Vector3f &at) {
	  Eigen::Matrix<float, Degree+1, 1> t; t.fill(0);
      typename Param<Degree>::VectorU pp = p;
      pp(0) -= at(2);
      
      for(int i=0; i<=Degree; i++)
        for(int j=0; j<=i; j++)
			for(int k=0; k<=j; k++)
				for(int kk=0; kk<=i-j; kk++)
					t(k+kk) += pp(i*(i+1)/2+j)*
					boost::math::binomial_coefficient<float>(j, k)*   std::pow(dir(0),k)* std::pow(off(0),j-k)*
					boost::math::binomial_coefficient<float>(i-j, kk)*std::pow(dir(1),kk)*std::pow(off(1),i-j-kk);

	  const Eigen::Matrix<float, Degree+1, Degree+1> M = t*t.transpose();
	  VectorU1D r;
	  r.fill(0);
	  
      for(int i=0; i<=Degree; i++)
		  for(int ii=0; ii<=Degree; ii++)
				r(i+ii) += M(i, ii);
			
	  const float x0=off(0)-at(0), y0=off(1)-at(1);	
	  r(0) += x0*x0 + y0*y0;
	  r(1) += 2*x0*dir(0) + 2*y0*dir(1);
	  r(2) += dir(0)*dir(0) + dir(1)*dir(1);
	  
      return r;
  }

  /**
   * check in z-axis with theshold (thr)
   */
  /*inline bool checkZ(const Param<Degree> &para, const float thr) const {
    float d=std::abs((para.model_.row(0)*p)(0)-para.z_(0));
    return d<thr*para.model_(0,0);
  }*/

  /**
   * check wether it could be linear, and convert it
   */
  bool isLinearAndTo()
  {
    if(Degree==1) return true;
    
    //Variance for both axis (spare small objects)
    const float Vx = x();
    const float Vy = y();

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

    float delta = 0.f;
    for(int i=3; i<p.cols(); i++)
      delta += std::abs(p(i));

    if(//std::abs(param.model_.determinant())*std::abs(param.z_(0))>0.0001f*param.model_(0,0)*param.model_(0,0) &&
        (error2 > (temp.param.model_(0,0)*temp.param.model_(0,0))/(1<<13)*d*d || (delta)/(std::abs(p(1))+std::abs(p(2)))>1.f))
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
  inline float get_max_gradient(const Param<Degree> &para) const {
    const float x = this->x();
    const float y = this->y();
    float dx=0.f, dy=0.f;
    for(int i=1; i<=Degree; i++)
      for(int j=0; j<=i; j++) {
        if(i!=j) dy += (i-j)* p(i*(i+1)/2+j)*std::pow(x,j) * std::pow(y,i-j-1);
        if(j!=0) dx += j* p(i*(i+1)/2+j)*std::pow(y,i-j) * std::pow(x,j-1);
      }

    return std::abs(dx) + std::abs(dy);
  }

  /**
   * approximation to nearest point from view point to check against
   */
  inline bool check_tangent(const Param<Degree> &para, const float thr) const {
    const float t=(para.z_(0)-(para.model_.row(0)*p)(0))/para.model_(0,0);

    const float x = this->x();
    const float y = this->y();
    float dx=0.f, dy=0.f;
    for(int i=1; i<=Degree; i++)
      for(int j=0; j<=i; j++) {
        //if(i!=j) dy += p(i*(i+1)/2+j)*(j-1<=0?1:std::pow(x,j-1))*(i-j-1<=0?1:std::pow(y,i-j-1));        //not the correct derivate (on purpose)
        //if(j!=0) dx += p(i*(i+1)/2+j)*(j-1<=0?1:std::pow(x,j-1))*(i-j-1<=0?1:std::pow(y,i-j-1));

        if(i!=j) dy += (i-j)*p(i*(i+1)/2+j)*std::pow(x,j) * std::pow(y,i-j-1);
        if(j!=0) dx += i*p(i*(i+1)/2+j)*std::pow(y,i-j) * std::pow(x,j-1);
      }

    Eigen::Vector3f r;
    r(0)=dx;
    r(1)=dy;
    r(2)=1.f;

    return t*t<thr*thr*r.squaredNorm();
  }

  /**
   * z-value at
   */
  float model(const float x, const float y) const {
    float r=0.f;
    for(int i=0; i<=Degree; i++)
      for(int j=0; j<=i; j++)
        r += p(i*(i+1)/2+j)*std::pow(x,j)*std::pow(y,i-j);
    return r;
  }

  /**
   * do regression for 6 parameters
   * TODO: if it's too steep (thr) make it linear
   */
  void get() {
    if(param.model_(0,0)==0) {
      p.fill(0.f);
    }
    else {
      p.head(Param<Degree>::NUM)=param.model_.topLeftCorner(Param<Degree>::NUM,Param<Degree>::NUM).fullPivLu().solve(param.z_.head(Param<Degree>::NUM));

      if(!pcl_isfinite(p(1))) {
        p.head(Param<Degree>::NUM)=param.model_.topLeftCorner(Param<Degree>::NUM,Param<Degree>::NUM).ldlt().solve(param.z_.head(Param<Degree>::NUM));

        if(!pcl_isfinite(p(1))) {
          ROS_ERROR("failure 0x0176");
          std::cerr<<param.model_<<"\n";
          std::cerr<<param.z_<<"\n";
        }
      }

    }
  }

  /// get 3 parameters for linear model
  Eigen::Vector3f getLinear() const {
    Eigen::Vector3f r;

    if(param.model_(0,0)!=0) {
      Eigen::Matrix3f M = param.model_.topLeftCorner(3,3);
      Eigen::Vector3f z = param.z_.head(3);
      r=M.fullPivLu().solve(z);
    }
    else r.fill(0);

    return r;
  }

  /// convert model to linear parameter setting
  void getLinear2()
  {
    Eigen::Vector3f r=getLinear();
    p.fill(0);
    p(0)=r(0);
    p(1)=r(1);
    p(2)=r(2);
  }

  /// not normalized normal at point (x,y)
  Eigen::Vector3f getNormal_(const float x, const float y) const {
    Eigen::Vector3f n;
    n(2)=1;
    n(0)=n(1)=0;
    for(int i=1; i<=Degree; i++)
      for(int j=0; j<=i; j++) {
        if(i!=j) n(1) -= (i-j)* p(i*(i+1)/2+j)*std::pow(x,j) * std::pow(y,i-j-1);
        if(j!=0) n(0) -= j* p(i*(i+1)/2+j)*std::pow(y,i-j) * std::pow(x,j-1);
      }
    return n;
  }

  /// normalized normal at point (x,y)
  Eigen::Vector3f getNormal(const float x, const float y) const {
    Eigen::Vector3f n = getNormal_(x,y);
    n.normalize();
    return n;
  }

  /// add model without update (call therefore get)
  void operator+=(const Model &m) {
    param+=m.param;
  }

  /// add node without update (call therefore get)
  void operator+=(const Param<Degree> &m) {
    param+=m;
  }

  /// ray cast surface with ray $z = (mx my 1)$ --> null points of polynomial equation (=roots)
  inline float intersect(const float mx, const float my) const {
    return intern::_intersect<Degree>(mx,my,p);
  }

  /// gets averaged x of current model (center point)
  inline float x() const {return param.model_(0,2)/param.model_(0,0);}
  /// gets averaged y of current model (center point)
  inline float y() const {return param.model_(0,1)/param.model_(0,0);}
  /// gets averaged z of current model (center point)
  inline float z() const {return param.z_(0)/param.model_(0,0);}
};

#ifdef QPPF_SPECIALIZATION_2
/**
 * specialization for second degree
 */
template <>
struct Model<2> {
  Param<2> param;
  typename Param<2>::VectorU p;

  Model() {p.fill(0.f);}
  Model(const Param<2> &pa) {p.fill(0.f); *this+=pa;}

  /**
   * check in z-axis with theshold (thr)
   */
  inline bool checkZ(const Param<2> &para, const float thr) const {
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
  inline float get_max_gradient(const Param<2> &para) const {
    return
        std::abs(2.f*para.model_(0,1)/para.model_(0,0)*p(2)+p(1)+para.model_(0,3)/para.model_(0,0)*p(5))
    + std::abs(2.f*para.model_(0,3)/para.model_(0,0)*p(4)+p(3)+para.model_(0,1)/para.model_(0,0)*p(5));
  }

  /**
   * approximation to nearest point from view point to check against
   */
  inline bool check_tangent(const Param<2> &para, const float thr) const {
    const float t=(para.z_(0)-(para.model_.row(0)*p)(0))/para.model_(0,0);
    //std::cout<<t<<" "<<(para.z_(0)/para.model_(0,0)-model(para.model_(0,1)/para.model_(0,0),para.model_(0,3)/para.model_(0,0)))<<" "<<thr<<"\n";
    Eigen::Vector3f r;
    r(0)=2.f*para.model_(0,1)/para.model_(0,0)*p(2)+p(1)+para.model_(0,3)/para.model_(0,0)*p(5);
    r(1)=2.f*para.model_(0,3)/para.model_(0,0)*p(4)+p(3)+para.model_(0,1)/para.model_(0,0)*p(5);
    r(2)=1.f;

    return t*t<thr*thr*r.squaredNorm();
  }

  /**
   * z-value at
   */
  /*float model(const Param<2> &para) const {
    //return (p(2)*para.model_(0,2)+p(4)*para.model_(0,4) + p(1)*para.model_(0,1)+p(3)*para.model_(0,3))/para.model_(0,0) + p(0);
    return (para.model_.row(0)*p)(0)/para.model_(0,0);
  }*/

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
      p.fill(0.f);
    }
    //    else if(param.model_(0,0)>1000 && std::abs(param.model_.determinant()/param.model_(0,0))<0.00001f)
    //    {
    //      getLinear2();
    //    }
    else {

      p.head<Param<2>::NUM>()=param.model_.topLeftCorner<Param<2>::NUM,Param<2>::NUM>().fullPivLu().solve(param.z_.head<Param<2>::NUM>());

      if(!pcl_isfinite(p(1))) {
        p.head<Param<2>::NUM>()=param.model_.topLeftCorner<Param<2>::NUM,Param<2>::NUM>().ldlt().solve(param.z_.head<Param<2>::NUM>());

        if(!pcl_isfinite(p(1))) {
          ROS_ERROR("failure 0x0176");
          std::cerr<<param.model_<<"\n";
          std::cerr<<param.z_<<"\n";
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
  void operator+=(const Param<2> &m) {
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

  inline float x() const {return param.model_(0,1)/param.model_(0,0);}
  inline float y() const {return param.model_(0,3)/param.model_(0,0);}
  inline float z() const {return param.z_(0)/param.model_(0,0);}
};
#endif

#endif /* MODEL_H_ */
