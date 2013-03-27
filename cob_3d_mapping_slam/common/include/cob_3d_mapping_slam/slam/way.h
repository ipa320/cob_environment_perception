/*
 * way.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef WAY_H_
#define WAY_H_


namespace Slam  /// namespace for all Slam related stuff
{

  template<typename NODE>
  struct SWAY
  {
    typedef typename NODE::DOF6 DOF6;

    typename NODE::Ptr node_;
    DOF6 link_;
    size_t id_;
  };
}



#endif /* WAY_H_ */
