/*
 * feature.h
 *
 *  Created on: 24.08.2012
 *      Author: josh
 */

#ifndef FEATURE_H_
#define FEATURE_H_


//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  struct S_FEATURE
  {
    enum TYPE {POINT, NORMAL, DIRECTION};

    int ID_;
    TYPE type_;
    Eigen::Vector3f v_, v_org_, n_;
    float var_, weight_;

    S_FEATURE(const cob_3d_mapping_msgs::feature &ft, const Eigen::Vector3f &n):var_(0.2f), weight_(1.f), n_(n) //TODO: default value?
    {
      ID_ = ft.ID;
      v_(0) = ft.x;
      v_(1) = ft.y;
      v_(2) = ft.z;
      v_org_=v_;
      switch(ft.ID)
      {
        case 2:
        case 4:
        case 5:
          v_.normalize();
          type_ = DIRECTION;
          break;
        case 3:
          type_ = POINT;
          break;
        default:
          ROS_ASSERT(0);
          break;
      }
      n_.normalize();
    }

    S_FEATURE(const Eigen::Vector3f &v, const Eigen::Vector3f &n, const bool bPlane):
      v_(v), var_(0.2f), weight_(1.f), n_(n) //TODO: default value?
    {
      type_ = bPlane?NORMAL:POINT;
      ID_ = 1;
      n_.normalize();
    }

    S_FEATURE(const Eigen::Vector3f &v, const Eigen::Vector3f &n, const float w):
      v_(v), var_(0.2f), weight_(w), n_(n) //TODO: default value?
    {
      type_ = POINT;
      ID_ = -1;
      n_.normalize();
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
    {
      switch(type_)
      {
        case POINT:
          var_ += v_.norm()*var_R+var_T;
          v_ = rot*v_ + tr;
          break;
        case NORMAL:
          var_ += v_.norm()*var_R+var_T; //TODO: should be middle not v
          v_ = rot*v_;
          if(tr.squaredNorm())
            v_ += v_*(tr.dot(v_))/(v_.squaredNorm()); //TODO: optimize
          break;
        case DIRECTION:
          v_ = rot*v_;
          var_ += var_R;
          break;
        default:
          ROS_ASSERT(0);
          break;
      }
      n_ = rot*n_;
    }

    bool isReachable(const S_FEATURE &o, const float thr_tr, const float thr_rot) const
    {
      if(type_ != o.type_)
      {
        ROS_INFO("cannot check features of differnt type");
        return false;
      }

      Eigen::Vector3f z;
      z(0)=z(1)=0;z(2)=1;
      switch(type_)
      {
        case POINT:
        {
          if(std::abs(o.v_.norm()-v_.norm())> thr_tr)
            return false;
          const float d = (o.v_-v_).norm();
          if(d>thr_tr && 2*std::asin( (d - thr_tr)/(2*v_.norm()) )>thr_rot)
            return false;
          if( std::acos( n_.dot(o.n_))>thr_rot + (n_.dot(z)+o.n_.dot(z))/12 )
            return false;
        }
          break;
        case NORMAL:
          if(std::abs(o.v_.norm()-v_.norm())> thr_tr)
            return false;
          if(2*std::asin( ((o.v_-v_).norm())/(2*v_.norm()) )>thr_rot)
            return false;
          break;
        case DIRECTION:
          if(2*std::asin( ((o.v_-v_).norm())/(2*v_.norm()) )>thr_rot)
            return false;
          break;

        default:
          ROS_ASSERT(0);
          break;
      }
      return true;
    }

    void merge(const S_FEATURE &o, const float weight, const float oweight)
    {
      ROS_ASSERT(var_>0);
      ROS_ASSERT(o.var_>0);
      if(type_ != o.type_)
      {
        ROS_INFO("cannot merge features of differnt type");
        return;
      }

      if(!pcl_isfinite(o.v_.sum())||!pcl_isfinite(o.var_))
        return;

      if(!pcl_isfinite(v_.sum())||!pcl_isfinite(var_))
      {
        v_ = o.v_;
        var_ = o.var_;
      }
      else
      {
        switch(type_)
        {
          case POINT:
          case NORMAL:
            //              std::cout<<"before\n"<<v_<<"\n\n"<<o.v_<<"\n";
            //              ROS_INFO("w %f %f %f %f", weight, oweight, var_,  o.var_);
            //v_ = v_ + (oweight*var_)/(weight*(var_+o.var_))*(o.v_-v_);
            v_ = v_ + (var_)/((var_+o.var_))*(o.v_-v_);
            n_ = n_ + o.n_;
            n_.normalize();
            //              std::cout<<"after\n"<<v_<<"\n";
            //              if(!pcl_isfinite(v_.sum())) ROS_INFO("HERE");
            break;
          case DIRECTION:
            //v_ = v_ + (oweight*var_)/(weight*(var_+o.var_))*(o.v_-v_);
            v_ = v_ + (var_)/((var_+o.var_))*(o.v_-v_);
            v_.normalize();
            break;

          default:
            ROS_ASSERT(0);
            break;
        }

        var_ = var_ - var_*var_/(var_+o.var_);
        //          std::cout<<"after\n"<<var_<<"\n";
      }

      //var_ = std::max(0.01f, var_);
    }

  };
}

#endif /* FEATURE_H_ */
