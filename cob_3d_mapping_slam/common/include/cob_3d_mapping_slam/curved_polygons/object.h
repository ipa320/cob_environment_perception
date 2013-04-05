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
    typedef Slam_CurvedPolygon::ex_curved_polygon DATA;

    //cob_3d_mapping_msgs::CurvedPolygon data_;
    DATA data_;
    size_t used_, creation_;
    bool proc_;

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

    Object(const DATA &cp):
      data_(cp), used_(1), creation_(1), proc_(true)
    {

    }

    const Slam_CurvedPolygon::ex_curved_polygon &getData() const {return data_;}

    inline Ptr makeShared () { return Ptr (new Object (*this)); }

    inline void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
    {
      data_.transform(rot, tr,var_R,var_T);
    }

    bool operator+=(const Object &o) {
      int st;
      return op_plus(o,st);
    }
    bool op_plus(const Object &o, int &status) {
      ///update
      if(data_.op_plus(o.data_,status) ) {
        used_ += o.used_;
        creation_ = std::max(creation_,o.creation_)+std::min(creation_,o.creation_)/2;
        if(used_>creation_)
          creation_ = used_;
        return true;
      }
      return false;
    }

    void used() {++used_;if(used_>creation_) --used_;}
    void processed() {++creation_;proc_=true;}
    void notProcessed() {proc_=false;}

    bool getProcessed() const {return proc_;}

    bool isReachable(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;
    std::vector<typename DOF6::TYPE> getDistance(const Object &o) const;

    bool operator|(const Object &o) const; /// check bounding box ... (wide match)
    bool operator&(const Object &o) const; /// check bounding box, size, ... and similarity(classification) (narrow match)
    TFLIST getTFList(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const;
    void addTFList(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, TFLIST &list) const;

    Eigen::Vector3f getNearestPoint() const {return data_.getNearestPoint();}
    Eigen::Vector3f getNearestTransformedPoint(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const {return data_.getNearestTransformedPoint(rot, tr);}

    size_t getUsedCounter() const {return used_;}
    size_t getCreationCounter() const {return creation_;}

    float getSimilarity(const Object &o) const;

    const DATA::BB &getBB() const;
    DATA::BB getBB(const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;

    /**
     * checks intersection of two bounding boxes
     *
     */
    bool intersectsBB(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;

    /**
     * checks intersection of bounding box and point
     *
     */
    bool intersectsBB(const Eigen::Vector3f &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;
    bool intersectsBB(const Eigen::Vector3f &o, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;
	
    bool intersectsPts(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const;

    bool invalid() const {
      return data_.getOutline().size()>100||data_.invalid();
    }
  };

#include "impl/object.hpp"

}


#endif /* OBJECT_H_ */
