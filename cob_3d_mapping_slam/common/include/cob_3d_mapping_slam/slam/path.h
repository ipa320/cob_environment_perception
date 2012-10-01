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
 * path.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef PATH_H_
#define PATH_H_

#include "way.h"

namespace Slam  /// namespace for all Slam related stuff
{

  template<typename NODE>
  class Path
  {
    typedef typename NODE::DOF6 DOF6;
    typedef typename NODE::OBJCTXT OBJCTXT;
    typedef typename OBJCTXT::OBJECT OBJECT;

    std::vector<SWAY<NODE> > path_;
    SWAY<NODE> local_;                        /// the one and only location where it is

    OBJCTXT act_ctxt_;                  /// stores object arangement from actual viewpoint (to register against)

    const typename DOF6::TYPE translation_res_, rotation_res_;

    double last_time_;                  /// last time stamp with wich start was called

    /**
     * check need for new node
     *
     * conditions:
     *  - probable distance ( |t| + v ) is below threshold
     *  - actual node has objects
     */
    bool needNewNode()
    {
      ROS_INFO("pot. tr  movement %f",local_.link_.getTranslation().norm()+local_.link_.getTranslationVariance());
      ROS_INFO("pot. rot movement %f",local_.link_.getRotation().norm()+local_.link_.getRotationVariance());
      if(
          (local_.node_->getNumObjs()>2) &&
          (
              //translation
              (local_.link_.getTranslation().norm()+local_.link_.getTranslationVariance() > translation_res_)
              ||
              //rotation
              (local_.link_.getRotation().norm()+local_.link_.getRotationVariance() > rotation_res_)
          )
      )
        return true;
      return false;
    }

    /// resets local node
    void newNode() {
      local_.node_.reset(new NODE());
      local_.link_ = DOF6();
    }

  public:

    Path(const typename DOF6::TYPE &thr_tr, const typename DOF6::TYPE &thr_rot)
    :
      translation_res_(thr_tr), rotation_res_(thr_rot)
    {
      newNode();
    }

    /*
     * 1. start frame (reseting local node)
     * 2. add all atom-nodes
     * 3. finish it, which will add tf-path between last node and local node
     */
    void startFrame(const double time_in_sec);
    void operator+=(typename OBJECT::Ptr obj);
    void finishFrame();

    const SWAY<NODE> &getLocal() const {return local_;}

  };

#include "impl/path.hpp"

}


#endif /* PATH_H_ */
