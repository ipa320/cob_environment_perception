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
 * registration.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef SCP_REGISTRATION_H_
#define SCP_REGISTRATION_H_

#include "../dof/tflink.h"
#include "object.h"

namespace Slam_CurvedPolygon
{
  template<typename _DOF6>
  class Registration
  {
  public:
    typedef _DOF6 DOF6;
    typedef Slam_CurvedPolygon::Object<DOF6> OBJECT;

  protected:

      /**
       * find correspondences for objects
       *   - search overlapping areas
       *   - search by similarity score
       */
    void findCorrespondences(const OBJCTXT &ctxt, DOF6 &tf, std::list<OBJECT::Ptr> &cors);
    

      /**
       * optimize link
       *   - remove non matching correspondences recursively
       */
    void optimizeLink(const OBJCTXT &ctxt, DOF6 &tf, std::list<OBJECT::Ptr> &cors, const TYPE &thr);


  public:

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate);


  };

#include "impl/registration.hpp"


#endif /* REGISTRATION_H_ */
