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
 * node.h
 *
 *  Created on: 26.05.2012
 *      Author: josh
 */

#ifndef NODE_H_
#define NODE_H_


#include "way.h"

namespace Slam
{

  /**
   * properties:
   *  - atom node is a seen object
   *  - non-atom node is a view-point
   *  - each atom node can be reached by max. 2 hops
   */
  //template<typename DATA, typename LINK>
  template<typename OBJECT_CONTEXT>
  class Node
  {
  public:
    typedef typename OBJECT_CONTEXT::DOF6 DOF6;
    typedef OBJECT_CONTEXT OBJCTXT;
    typedef typename OBJCTXT::OBJECT OBJECT;

    typedef boost::shared_ptr<Node> Ptr;

  private:

    std::vector<SWAY<Node> > connections_;      /// connections to other nodes
    OBJCTXT ctxt_;                              /// context includes all operators for objects (registration, correspondences,...) + objects

  public:

    void addLink(const SWAY<Node> &con) {
      connections_.push_back(con);
    }

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate);
    bool merge(const OBJCTXT &ctxt, DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const bool only_merge);

    bool addCtxt(const OBJCTXT &ctxt, const DOF6 &tf);

    size_t getNumObjs() const {return ctxt_.getNumObjs();}

    const std::vector<SWAY<Node> > &getConnections() const {return connections_;}

    const OBJECT_CONTEXT &getContext() const {return ctxt_;}

    bool compute(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr,bool> &used, const bool only_merge=false, const int depth=0);
  };

#include "impl/node.hpp"

}


#endif /* NODE_H_ */
