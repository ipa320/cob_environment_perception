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

    struct TransformedFoVBB
    {
      struct Plane {
        Eigen::Vector3f normal_x_,normal_y_, offs_;
        Eigen::Vector2f boundary_[4];

        int pnpoly(float x, float y) const
        {
          int i, j, c = 0;
          for (i = 0, j = 4-1; i < 4; j = i++) {
            if ((((boundary_[i](1) <= y) && (y < boundary_[j](1))) ||
                ((boundary_[j](1) <= y) && (y < boundary_[i](1)))) &&
                (x < (boundary_[j](0) - boundary_[i](0)) * (y - boundary_[i](1)) / (boundary_[j](1) - boundary_[i](1)) + boundary_[i](0)))
              c = !c;
          }
          return c;
        }

        bool intersectRay(const Eigen::Vector3f &a, const Eigen::Vector3f &b) const
        {
          Eigen::Matrix3f M;
          Eigen::Vector3f v;
          M.col(0) = a-b;
          M.col(1) = normal_x_;
          M.col(2) = normal_y_;
          v = M.inverse()*(a-offs_);
          if(std::abs(v(0))<=1)
          {
            return pnpoly(v(1),v(2));
          }
          return false;
        }

      };

      Plane planes_[6];
      Eigen::Vector3f p_[8];

      bool check(const TransformedFoVBB &o) const {
        for(int j=0; j<6; j++)
          for(int i=0; i<4; i++)
          {
            if(planes_[j].intersectRay(p_[i],p_[(i+1)%4]))
              return true;
            if(planes_[j].intersectRay(p_[i+4],p_[(i+1)%4+4]))
              return true;
            if(planes_[j].intersectRay(p_[i+4],p_[i]))
              return true;
          }
        return false;
      }

      bool operator&(const TransformedFoVBB &o) const
      {
        return check(o) || o.check(*this);
      }
    };

    struct FoVBB ///Field of View Bounding Box (as pyramid)
    {
      typename DOF6::TYPE min_dist_, max_dist_;
      typename DOF6::TYPE min_x_, min_y_, max_x_, max_y_;

      void update(const OBJCTXT &ctxt) {
        min_dist_ = min_x_ = min_y_ = std::numeric_limits<float>::max();
        max_dist_ = max_x_ = max_y_ = std::numeric_limits<float>::min();

        for(size_t i=0; i<ctxt.getObjs().size(); i++) {
          Eigen::Vector3f mi, ma;
          ctxt.getObjs()[i]->getData().getBB(mi,ma);
          min_dist_ = std::min(min_dist_, mi(2));
          min_x_ = std::min(min_x_, mi(0)/mi(2));
          min_y_ = std::min(min_y_, mi(1)/mi(2));
          max_dist_ = std::max(max_dist_, ma(2));
          max_x_ = std::max(max_x_, ma(0)/mi(2));
          max_y_ = std::max(max_y_, ma(1)/mi(2));
        }
      }

      TransformedFoVBB transform(const Eigen::Matrix3f &R, const Eigen::Vector3f t) const {
        ROS_INFO("bb %f %f ",min_dist_,max_dist_);

        TransformedFoVBB r;
        r.p_[0](0) = min_x_ * min_dist_;
        r.p_[0](1) = min_y_ * min_dist_;
        r.p_[1](0) = max_x_ * min_dist_;
        r.p_[1](1) = min_y_ * min_dist_;
        r.p_[2](0) = max_x_ * min_dist_;
        r.p_[2](1) = max_y_ * min_dist_;
        r.p_[3](0) = min_x_ * min_dist_;
        r.p_[3](1) = max_y_ * min_dist_;

        r.p_[4](0) = min_x_ * max_dist_;
        r.p_[4](1) = min_y_ * max_dist_;
        r.p_[5](0) = max_x_ * max_dist_;
        r.p_[5](1) = min_y_ * max_dist_;
        r.p_[6](0) = max_x_ * max_dist_;
        r.p_[6](1) = max_y_ * max_dist_;
        r.p_[7](0) = min_x_ * max_dist_;
        r.p_[7](1) = max_y_ * max_dist_;

        Eigen::Vector3f x,y,z;
        x=y=z=Eigen::Vector3f::Zero();
        x(0)=1;
        y(1)=1;
        z(2)=1;

        r.planes_[0].offs_ = R*min_dist_*z+t;
        r.planes_[1].offs_ = R*max_dist_*z+t;
        r.planes_[1].normal_x_ = r.planes_[0].normal_x_ = R*x;
        r.planes_[1].normal_y_ = r.planes_[0].normal_y_ = R*y;
        for(int i=0; i<4; i++)
        {
          r.planes_[0].boundary_[i] = (r.p_[i].head(2));
          r.planes_[1].boundary_[i] = (r.p_[i+4].head(2));
        }

        Eigen::Vector3f p[8];
        for(int i=0; i<8; i++)
        {
          r.p_[i](2) = i<4 ? min_dist_:max_dist_;
          p[i] = r.p_[i];
          r.p_[i] = R*r.p_[i] + t;
        }

        for(int i=0; i<4; i++)
        {
          r.planes_[2+i].normal_x_ = r.p_[i]-r.p_[i+4];
          r.planes_[2+i].normal_x_.normalize();
          r.planes_[2+i].normal_y_ = r.p_[i]-r.p_[(i+1)%4+4];
          r.planes_[2+i].normal_y_.normalize();

          r.planes_[2+i].boundary_[0] = p[i].head<2>();
          r.planes_[2+i].boundary_[1] = p[(i+1)%4].head<2>();
          r.planes_[2+i].boundary_[2] = p[(i+1)%4+4].head<2>();
          r.planes_[2+i].boundary_[3] = p[i+4].head<2>();

          r.planes_[2+i].offs_ = t;
        }

        return r;
      }

    };

  private:

    struct SCOR
    {
      typename OBJECT::Ptr a,b;
      bool used_;
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
    std::list<SCOR> used_cors_; ///remeber successfully used correspondences from registration for merging
    size_t frames_;
    FoVBB bb_;                                  /// allows fast intersection tests

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
    DOF6 optimizeLink(const DOF6 &tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr /*=std::numeric_limits<typename DOF6::TYPE>::quiet_NaN()*/, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr,
                      const int depth=0) const;


    /// remove "noise" like seldom seend objects
    void filter();

    /// update the bounding box (pyramid)
    void updateBB();

  public:

    OBJCTXT():frames_(0)
    {}

    void operator+=(typename OBJECT::Ptr obj) {
      //ROS_ASSERT(!obj->getData().invalid());
      objs_.push_back(obj);
    }

    void clear() {objs_.clear();}

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate);

    bool merge(const OBJCTXT &ctxt, const DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const bool only_merge);

    /// update all properties: 1. filter  2. bounding box
    void update();

    size_t getNumObjs() const {return objs_.size();}

    const std::vector<typename OBJECT::Ptr> &getObjs() const {return objs_;}

    size_t getFramesProcessed() const {return frames_;}

    const FoVBB &getBoundingBox() const {return bb_;}

    bool empty() const {return objs_.size()==0;}

  };

#include "impl/objctxt.hpp"
#include "impl/registration.hpp"

}



#endif /* OBJCTXT_H_ */
