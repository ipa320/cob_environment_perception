/*
 * objctxt.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef OBJCTXT_H_
#define OBJCTXT_H_

#include "object.h"

namespace Dummy
{
  template<typename _DOF6>
  class OBJCTXT
  {
  public:
    typedef Dummy::Object OBJECT;
    typedef _DOF6 DOF6;

    void clear() {}
    void operator+=(OBJECT::Ptr obj) {}

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate){
      return false;
    }

    bool add(const OBJCTXT &ctxt, const DOF6 &tf){
      return false;
    }

    size_t getNumObjs() const {return 0;}

  };
}



#endif /* OBJCTXT_H_ */
