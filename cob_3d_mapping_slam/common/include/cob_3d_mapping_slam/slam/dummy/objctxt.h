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
