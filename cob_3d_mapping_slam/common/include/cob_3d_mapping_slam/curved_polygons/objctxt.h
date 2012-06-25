/*
 * objctxt.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef SCP_OBJCTXT_H_
#define SCP_OBJCTXT_H_

#include "../dof/tflink.h"
#include "object.h"

namespace Slam_CurvedPolygon
{
  template<typename _DOF6>
  class OBJCTXT
  {
  public:
    typedef _DOF6 DOF6;
    typedef Slam_CurvedPolygon::Object<DOF6> OBJECT;

  private:

    std::vector<typename OBJECT::Ptr> objs_;

    struct SCOR
    {
      typename OBJECT::Ptr a,b;
    };

    struct SCOR_DISTANCES;

    struct SCOR_MEMBER
    {
      typedef boost::shared_ptr<SCOR_MEMBER> Ptr;
      typedef std::map<SCOR_MEMBER::Ptr, std::vector<typename DOF6::TYPE> > MAP;

      typename OBJECT::Ptr obj_;
      std::vector<SCOR_MEMBER::Ptr> candidates_;
      MAP distances_;
    };

    typename DOF6::TYPE check_assumption(typename SCOR_MEMBER::Ptr m1, typename SCOR_MEMBER::Ptr m2) const;

    /**
     * find correspondences for objects
     *   - search overlapping areas
     *   - search by similarity score
     */
    void findCorrespondences(const OBJCTXT &ctxt, std::list<SCOR> &cors,
                             const DOF6 &tf) const;


    /**
     * optimize link
     *   - remove non matching correspondences recursively
     */
    DOF6 optimizeLink(const DOF6 &tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr /*=std::numeric_limits<typename DOF6::TYPE>::quiet_NaN()*/, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const int depth=0) const;


  public:

    void clear() {objs_.clear();}

    void operator+=(typename OBJECT::Ptr obj) {
      objs_.push_back(obj);
    }

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate);

    bool add(const OBJCTXT &ctxt, const DOF6 &tf);

    size_t getNumObjs() const {return objs_.size();}

  };

#include "impl/objctxt.hpp"
#include "impl/registration.hpp"

}



#endif /* OBJCTXT_H_ */
