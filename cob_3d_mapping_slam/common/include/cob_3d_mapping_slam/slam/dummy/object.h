/*
 * object.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef OBJECT_H_
#define OBJECT_H_

namespace Dummy
{
  class Object
  {
  public:
    typedef boost::shared_ptr<Object> Ptr;

    inline Ptr makeShared () { return Ptr (new Object (*this)); }
  };
}


#endif /* OBJECT_H_ */
