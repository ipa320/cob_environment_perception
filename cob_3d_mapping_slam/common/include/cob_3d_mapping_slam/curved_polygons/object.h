/*
 * object.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef SCP_OBJECT_H_
#define SCP_OBJECT_H_

#include "extended_curved_polygon.h"

namespace Slam_CurvedPolygon
{

  template<typename _DOF6>
  class Object
  {
    //cob_3d_mapping_msgs::CurvedPolygon data_;
    Slam_CurvedPolygon::ex_curved_polygon data_;

  public:

    typedef _DOF6 DOF6;
    typedef typename DOF6::SOURCE1 TFLINK;

    struct TF_CORS
    {
      typename TFLINK::TFLinkObj a,b;

      TF_CORS(const typename TFLINK::TFLinkObj &a, const typename TFLINK::TFLinkObj &b):
        a(a), b(b)
      {}
    };

    typedef boost::shared_ptr<Object> Ptr;
    typedef typename std::list<TF_CORS> TFLIST;

    Object(const cob_3d_mapping_msgs::CurvedPolygon &cp):
      data_(cp)
    {

    }

    const Slam_CurvedPolygon::ex_curved_polygon &getData() const {return data_;}

    inline Ptr makeShared () { return Ptr (new Object (*this)); }

    inline void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
    {
      data_.transform(rot, tr,var_R,var_T);
    }

    bool operator+=(const Object &o) {//TODO:
      ///update
      //data_ = o.data_;
      data_ += o.data_;
      return true;
    }

    bool isReachable(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;
    std::vector<typename DOF6::TYPE> getDistance(const Object &o) const;

    bool operator|(const Object &o) const; /// check bounding box ... (wide match)
    bool operator&(const Object &o) const; /// check bounding box, size, ... and similarity(classification) (narrow match)
    TFLIST getTFList(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const;

    Eigen::Vector3f getNearestPoint() const {return data_.getNearestPoint();}
    Eigen::Vector3f getNearestTransformedPoint(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const {return data_.getNearestTransformedPoint(rot, tr);}

  };

#include "impl/object.hpp"

}


#endif /* OBJECT_H_ */
