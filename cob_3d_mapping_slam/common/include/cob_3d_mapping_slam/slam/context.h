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
 * context.h
 *
 *  Created on: 26.05.2012
 *      Author: josh
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include "path.h"
#include "hypothesis.h"

namespace Slam  /// namespace for all Slam related stuff
{

  /**
   * maps between atom-nodes and non-atom-nodes
   */
  template<typename KEY, typename NODE>
  class Context
  {
    typedef typename NODE::DOF6 DOF6;
    typedef typename NODE::OBJCTXT OBJCTXT;
    typedef typename OBJCTXT::OBJECT OBJECT;
    typedef typename Slam::Path<NODE> PATH;
    typedef typename Slam::Hypothesis<NODE> HYPO;

    typedef std::map<typename KEY::TYPE,std::list<typename NODE::Ptr> > MAP;
    MAP map_;

    PATH path_;

    std::vector<HYPO> hypothesis_; /// as long as the position is uncertain each possible path will be calculated and stored here


  public:

    Context(const typename DOF6::TYPE &thr_tr, const typename DOF6::TYPE &thr_rot)
    : path_(thr_tr,thr_rot)
    {

    }

    /*
     * 1. start frame (reseting local node)
     * 2. add all atom-nodes
     * 3. finish it, which will add tf-path between last node and local node
     */
    void startFrame(const double time_in_sec);
    void operator+=(typename OBJECT::Ptr obj);
    void finishFrame();

    void operator+=(OBJECT obj)
    {
      *this += obj.makeShared();
    }

    const PATH &getPath() const {return path_;}
    PATH &getPath() {return path_;}
  };

#include "impl/context.hpp"

}


#endif /* CONTEXT_H_ */
