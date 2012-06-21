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
