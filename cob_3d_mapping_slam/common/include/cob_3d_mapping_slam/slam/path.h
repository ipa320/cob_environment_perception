/*
 * path.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef PATH_H_
#define PATH_H_

#include "way.h"

namespace Slam  /// namespace for all Slam related stuff
{

  template<typename NODE>
  class Path
  {
    typedef typename NODE::DOF6 DOF6;
    typedef typename NODE::OBJCTXT OBJCTXT;
    typedef typename OBJCTXT::OBJECT OBJECT;

    std::vector<SWAY<NODE> > path_;
    SWAY<NODE> local_;                        /// the one and only location where it is

    OBJCTXT act_ctxt_;                  /// stores object arangement from actual viewpoint (to register against)

    const typename DOF6::TYPE translation_res_, rotation_res_;

    double last_time_;                  /// last time stamp with wich start was called

    /**
     * check need for new node
     *
     * conditions:
     *  - probable distance ( |t| + v ) is below threshold
     *  - actual node has objects
     */
    bool needNewNode()
    {
      ROS_INFO("pot. tr  movement %f",local_.link_.getTranslation().norm()+local_.link_.getTranslationVariance());
      ROS_INFO("pot. rot movement %f",local_.link_.getRotation().norm()+local_.link_.getRotationVariance());
      if(
          (local_.node_->getNumObjs()>2) &&
          (
              //translation
              (local_.link_.getTranslation().norm()+local_.link_.getTranslationVariance() > translation_res_)
              ||
              //rotation
              (local_.link_.getRotation().norm()+local_.link_.getRotationVariance() > rotation_res_)
          )
      )
        return true;
      return false;
    }

    /// resets local node
    void newNode() {
      static size_t id = 0;
      local_.node_.reset(new NODE());
      local_.link_ = DOF6();
      local_.id_ = id;
      id++;
    }

  public:

    Path(const typename DOF6::TYPE &thr_tr, const typename DOF6::TYPE &thr_rot)
    :
      translation_res_(thr_tr), rotation_res_(thr_rot)
    {
      newNode();
    }

    /*
     * 1. start frame (reseting local node)
     * 2. add all atom-nodes
     * 3. finish it, which will add tf-path between last node and local node
     */
    void startFrame(const double time_in_sec);
    void operator+=(typename OBJECT::Ptr obj);
    void finishFrame();

    const SWAY<NODE> &getLocal() const {return local_;}
    SWAY<NODE> &getLocal() {return local_;}

    bool getTF(DOF6 &tf, const SWAY<NODE> *start, const SWAY<NODE> *end);
  };

#include "impl/path.hpp"

}


#endif /* PATH_H_ */
