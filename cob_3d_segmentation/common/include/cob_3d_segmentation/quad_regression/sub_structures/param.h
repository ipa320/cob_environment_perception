/*
 * param.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef PARAM_H_
#define PARAM_H_


/**
 * leaf of quad-tree
 * contains data for regression of 6 parameters
 */
struct Param {
  typedef Eigen::Matrix<float, 6, 1>    Vector6f;
  typedef Eigen::Matrix<float, 6, 6>    Matrix6f;

  Matrix6f model_;        /// regression data
  Vector6f z_;              /// result vector
#ifdef USE_MIN_MAX_RECHECK_
  float v_min_,v_max_;
#endif
  int occopied;             /// mark

  /// constructor, sets everything to 0
  Param() {
    for(int i=0; i<6; i++)
    {z_(i)=0; for(int j=0; j<6; j++) model_(i,j)=0;}
  }

  /// init. with point
  inline void operator=(const Eigen::Vector3f &p) {
#ifdef USE_MIN_MAX_RECHECK_
    v_min_=v_max_=p(2);
#endif

    if(pcl_isfinite(p(2))) {

      float x=p(0);
      float x2=x*x;
      float y=p(1);
      float y2=y*y;

      z_(0) = p(2);
      z_(1) = p(2)*x;
      z_(2) = p(2)*x2;
      z_(3) = p(2)*y;
      z_(4) = p(2)*y2;
#ifndef DONT_USE_6TH
z_(5) = p(2)*x*y;
#endif

for(int i=0; i<6; i++) {
  float m=1;
  switch(i) {
    case 1: m=x;break;
    case 2: m=x2;break;
    case 3: m=y;break;
    case 4: m=y2;break;
#ifndef DONT_USE_6TH
    case 5: m=x*y;break;
#else
    case 5: m=0.f;break;
#endif
  }
  model_(i,0)=m;
  model_(i,1)=x*m;
  model_(i,2)=x2*m;
  model_(i,3)=y*m;
  model_(i,4)=y2*m;
#ifndef DONT_USE_6TH
  model_(i,5)=x*y*m;
#endif
}

    }
    else {
#ifdef USE_MIN_MAX_RECHECK_
      v_min_=v_max_=0.f;
#endif
      z_(0)=z_(1)=z_(2)=z_(3)=z_(4)=z_(5)=0.f;
      for(int i=0; i<6; i++) for(int j=0; j<6; j++)
        model_(i,j)=0.f;
    }
  }

  /// add point
  inline void operator+=(const Eigen::Vector3f &p) {
    if(pcl_isfinite(p(2))) {
#ifdef USE_MIN_MAX_RECHECK_
      if(p(2)<v_min_||v_min_==0.f) v_min_=p(2);
      if(p(2)>v_max_||v_max_==0.f) v_max_=p(2);
#endif

      float x=p(0);
      float x2=p(0)*p(0);
      float y=p(1);
      float y2=p(1)*p(1);

      z_(0) += p(2);
      z_(1) += p(2)*x;
      z_(2) += p(2)*x2;
      z_(3) += p(2)*y;
      z_(4) += p(2)*y2;
#ifndef DONT_USE_6TH
      z_(5) += p(2)*x*y;
#endif

      for(int i=0; i<5; i++) {
        float m=1;
        switch(i) {
          case 1: m=x;break;
          case 2: m=x2;break;
          case 3: m=y;break;
          case 4: m=y2;break;
#ifndef DONT_USE_6TH
          case 5: m=x*y;break;
#else
          case 5: m=0.f;break;
#endif
        }
        model_(i,0)+=m;
        model_(i,1)+=x*m;
        model_(i,2)+=x2*m;
        model_(i,3)+=y*m;
        model_(i,4)+=y2*m;
#ifndef DONT_USE_6TH
        model_(i,5)+=x*y*m;
#endif
      }

    }
  }

  ///add leave
  inline void operator+=(const Param &p) {
    model_+=p.model_;
    z_+=p.z_;
#ifdef USE_MIN_MAX_RECHECK_
    if((p.v_min_!=0.f && p.v_min_<v_min_)||v_min_==0.f) v_min_=p.v_min_;
    if(p.v_max_>v_max_||v_max_==0.f) v_max_=p.v_max_;
#endif
  }
};

/**
 * contains level of quad-tree
 * w: width
 * h: height
 * data: nodes/leaves
 */
struct ParamC {
  Param *data;
  unsigned int w,h;

  ParamC(const ParamC &p):w(p.w),h(p.h) {
    data=new Param[w*h];
    for(size_t i=0; i<w*h; i++)
      data[i]=p.data[i];
  }

  ParamC(unsigned int w, unsigned int h):w(w),h(h) {
    data=new Param[w*h];
  }

  ~ParamC() {
    delete [] data;
  }

};


#endif /* PARAM_H_ */
