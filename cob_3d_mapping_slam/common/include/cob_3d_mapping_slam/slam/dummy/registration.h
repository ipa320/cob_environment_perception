/*
 * registration.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef REGISTRATION_H_
#define REGISTRATION_H_


namespace Dummy
{

  template<typename OBJCTXT, typename DOF6>
  bool registration_dummy(const OBJCTXT &, const OBJCTXT &, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate)
  {
    return true;
  }

}


#endif /* REGISTRATION_H_ */
