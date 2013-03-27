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
#include "bb.h"

namespace Slam_CurvedPolygon
{
  template<typename _DOF6>
  class OBJCTXT
  {
  public:
    typedef boost::shared_ptr<OBJCTXT> Ptr;
    typedef _DOF6 DOF6;
    typedef Slam_CurvedPolygon::Object<DOF6> OBJECT;


    typedef class CtxtBB : public BoundingBox::FoVBB<typename DOF6::TYPE>
    {
    public:
      void update(const OBJCTXT &ctxt) {
        this->min_dist_ = this->min_x_ = this->min_y_ =  std::numeric_limits<typename DOF6::TYPE>::max();
        this->max_dist_ = this->max_x_ = this->max_y_ = -std::numeric_limits<typename DOF6::TYPE>::max();

        for(size_t i=0; i<ctxt.getObjs().size(); i++) {
          for(size_t j=0; j<ctxt.getObjs()[i]->getData().getPoints3D().size(); j++) {
            const Eigen::Vector3f &v = ctxt.getObjs()[i]->getData().getPoints3D()[j];
            this->min_dist_ = std::min(this->min_dist_, v(2));
            this->min_x_ = std::min(this->min_x_, v(0)/v(2));
            this->min_y_ = std::min(this->min_y_, v(1)/v(2));
            this->max_dist_ = std::max(this->max_dist_, v(2));
            this->max_x_ = std::max(this->max_x_, v(0)/v(2));
            this->max_y_ = std::max(this->max_y_, v(1)/v(2));
          }
        }

        ROS_INFO("BB (%f %f) (%f %f) (%f %f)",this->min_dist_,this->max_dist_, this->min_x_,this->max_x_, this->min_y_,this->max_y_);
      }
    } BB;

    private:

    struct SCOR
    {
      typename OBJECT::Ptr a,b;
      bool used_;
      float prob;
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


    std::vector<typename OBJECT::Ptr> objs_;
    std::vector<SCOR> used_cors_; ///remeber successfully used correspondences from registration for merging
    std::map<typename OBJECT::Ptr,std::vector<size_t> > map_cors_;
    size_t frames_;
    BB bb_;                                  /// allows fast intersection tests

    //paramters
    bool enabled_all_merge_;

    typename DOF6::TYPE check_assumption(typename SCOR_MEMBER::Ptr m1, typename SCOR_MEMBER::Ptr m2) const;

    /**
     * find correspondences for objects
     *   - search overlapping areas
     *   - search by similarity score
     */
    void findCorrespondences1(const OBJCTXT &ctxt, std::list<SCOR> &cors,
                              const DOF6 &tf) const;
    void findCorrespondences3(const OBJCTXT &ctxt, std::vector<SCOR> &cors,
                              const DOF6 &tf);


    /**
     * optimize link 1
     *   - remove non matching correspondences recursively
     *   - takes all correspondences in account
     */
    DOF6 optimizeLink1(const DOF6 &tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr /*=std::numeric_limits<typename DOF6::TYPE>::quiet_NaN()*/, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr,
                       const int depth=0) const;

    /**
     * optimize link 2
     *   - remove non matching correspondences recursively
     *   - only uses most probable correspondences
     */
    DOF6 optimizeLink2(const DOF6 &tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr /*=std::numeric_limits<typename DOF6::TYPE>::quiet_NaN()*/, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr,
                       const int depth=0) const;

    DOF6 optimizeLink3(const OBJCTXT &ctxt, std::vector<SCOR> &cors, const DOF6 &tf, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr /*=std::numeric_limits<typename DOF6::TYPE>::quiet_NaN()*/, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr,
                       const int depth=0) const;


    /// remove "noise" like seldom seend objects
    void filter();

    /// update the bounding box (pyramid)
    void updateBB();

    public:

    OBJCTXT():frames_(0), enabled_all_merge_(true)
    {}

    void operator+=(typename OBJECT::Ptr obj) {
//      if(!obj->getData().invalid())
//        ROS_ERROR("invaild object");
      objs_.push_back(obj);
    }

    void operator+=(const OBJCTXT &o) {
      for(size_t i=0; i<o.objs_.size(); i++)
        objs_.push_back(o.objs_[i]);
      updateBB();
    }

    void clear() {objs_.clear();}

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate);

    bool merge(const OBJCTXT &ctxt, const DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const BoundingBox::TransformedFoVBB &fov, const bool only_merge);

    /// update all properties: 1. filter  2. bounding box
    void update();

    size_t getNumObjs() const {return objs_.size();}

    const std::vector<typename OBJECT::Ptr> &getObjs() const {return objs_;}

    size_t getFramesProcessed() const {return frames_;}

    const BB &getBoundingBox() const {return bb_;}

    bool empty() const {return objs_.size()==0;}

    typename OBJCTXT::Ptr clone() const;
    OBJCTXT &transform(const DOF6 &tf);

  };

#include "impl/objctxt.hpp"
#include "impl/registration.hpp"

}



#endif /* OBJCTXT_H_ */
