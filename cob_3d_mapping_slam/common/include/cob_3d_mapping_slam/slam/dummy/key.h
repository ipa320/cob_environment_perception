/*
 * key.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef KEY_H_
#define KEY_H_

#include "object.h"

namespace Dummy
{
  class KEY
  {
  public:
    typedef int TYPE;
    typedef std::list<TYPE> KEYS; //list to test some others

    KEY(typename Dummy::Object::Ptr obj)
    {}

    KEYS getKeys() {
      KEYS r;
      return r;
    }

  };
}



#endif /* KEY_H_ */
