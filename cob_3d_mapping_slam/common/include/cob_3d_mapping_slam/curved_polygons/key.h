/*
 * key.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef SCP_KEY_H_
#define SCP_KEY_H_

#include "object.h"

namespace Slam_CurvedPolygon
{
  template<typename _DOF6>
  class KEY
  {
    typedef typename Slam_CurvedPolygon::Object<_DOF6> OBJECT;
    typename OBJECT::Ptr obj_;
  public:
    typedef unsigned int TYPE;
    typedef std::vector<TYPE> KEYS;

    KEY(typename OBJECT::Ptr obj)
    {
      obj_ = obj;
    }

    KEYS getKeys() {
      KEYS r;

      if(!obj_)
        return r;

      for(size_t i=0; i<obj_->getData().getScore().size(); i++)
        r.push_back( obj_->getData().getScore()[i].ID );
      return r;
    }
  };
}



#endif /* KEY_H_ */
