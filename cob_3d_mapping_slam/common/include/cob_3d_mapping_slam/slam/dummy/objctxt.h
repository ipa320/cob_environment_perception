/*
 * objctxt.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef OBJCTXT_H_
#define OBJCTXT_H_

#include "object.h"
#include "../../curved_polygons/bb.h"

namespace Dummy
{
  template<typename _DOF6>
  class OBJCTXT
  {
  public:
    typedef Dummy::Object OBJECT;
    typedef _DOF6 DOF6;
    typedef boost::shared_ptr<OBJCTXT> Ptr;
    typedef BoundingBox::FoVBB<float> BB;

    BB bb_;
    int ctr_;

    OBJCTXT(): ctr_(1) {}

    void clear() {}
    void update() {}
    bool empty() const {return false;}
    void operator+=(OBJECT::Ptr obj) {}
    void operator+=(const OBJCTXT &obj) {}

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate){
      typename DOF6::SOURCE1 &magic_box(*tf.getSource1());
      magic_box.reset();

      Eigen::Vector3f t=Eigen::Vector3f::Zero(),dt=Eigen::Vector3f::Zero(),v;
      Eigen::Matrix3f R=Eigen::Matrix3f::Identity();

      Eigen::AngleAxisf aa(0.1f,Eigen::Vector3f::Identity());
      dt(0) = 0.2f;

      for(int i=0; i<ctr_; i++) {
        t += dt;
        R = aa*R;
      }
      ++ctr_;

      v(0)=1;v(1)=v(2)=0;
      magic_box(
          typename DOF6::SOURCE1::TFLinkObj(v, false,false, 1, 1),
          typename DOF6::SOURCE1::TFLinkObj(R*v+t,false,false, 1, 1)
            );

      v(1)=1;v(0)=v(2)=0;
      magic_box(
          typename DOF6::SOURCE1::TFLinkObj(v, false,false, 1, 1),
          typename DOF6::SOURCE1::TFLinkObj(R*v+t,false,false, 1, 1)
            );

      v(2)=1;v(1)=v(0)=0;
      magic_box(
          typename DOF6::SOURCE1::TFLinkObj(v, false,false, 1, 1),
          typename DOF6::SOURCE1::TFLinkObj(R*v+t,false,false, 1, 1)
            );

      magic_box.finish();

      return true;//ctr_<5;
    }

    bool merge(const OBJCTXT &ctxt, const DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const BoundingBox::TransformedFoVBB &fov, const bool only_merge) {
      return false;
    }

    size_t getNumObjs() const {return 10;}

    const BB getBoundingBox() const {return bb_;}

    OBJCTXT &transform(const DOF6 &tf) {}
    typename OBJCTXT::Ptr clone() const {return typename OBJCTXT::Ptr(new OBJCTXT(*this));}

  };
}



#endif /* OBJCTXT_H_ */
