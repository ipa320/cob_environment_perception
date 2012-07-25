/*
 * object.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef SCP_EXTENDED_CURVED_POLYGON_H_
#define SCP_EXTENDED_CURVED_POLYGON_H_

#define DEBUG_LEVEL 30

#include <cob_3d_mapping_msgs/CurvedPolygon.h>
#include "/home/josh/workspace/dynamic_tutorials/common/include/registration/object.hpp"
//#include "sac_model_correspondence.h"
#include "form.h"
#include "polygon_merger.h"
#include <unsupported/Eigen/NonLinearOptimization>


namespace Slam_CurvedPolygon
{

  class ex_curved_polygon : private cob_3d_mapping_msgs::CurvedPolygon
  {
  public:

    /**
     * S_ID contains more than one id
     * different ids can mean same object, but this can only be recognized after registration
     */
    struct S_ID { //TODO: replace with sorted list
      std::vector<int> ids_;

      void operator+=(const int id) {
        if(!(*this==id))
          ids_.push_back(id);
      }

      void operator+=(const S_ID &o) {
        for(size_t i=0; i<o.ids_.size(); i++)
          *this += o.ids_[i];
      }

      bool operator==(const int id) const {
        for(size_t i=0; i<ids_.size(); i++)
          if(ids_[i]==id)
            return true;
        return false;
      }

      bool operator==(const S_ID &o) const {
        for(size_t i=0; i<ids_.size(); i++)
          if(o==ids_[i])
            return true;
        return false;
      }
    };

    struct S_FEATURE
    {
      enum TYPE {POINT, NORMAL, DIRECTION};

      int ID_;
      TYPE type_;
      Eigen::Vector3f v_, v_org_;
      float var_;

      S_FEATURE(const cob_3d_mapping_msgs::feature &ft):var_(0.2f) //TODO: default value?
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
      }

      S_FEATURE(const Eigen::Vector3f &v, const bool bPlane):
        v_(v), var_(0.2f) //TODO: default value?
      {
        type_ = bPlane?NORMAL:POINT;
        ID_ = 1;
      }

      S_FEATURE(const Eigen::Vector3f &v):
        v_(v), var_(0.2f) //TODO: default value?
      {
        type_ = POINT;
        ID_ = -1;
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
              std::cout<<"before\n"<<v_<<"\n\n"<<o.v_<<"\n";
              ROS_INFO("w %f %f %f %f", weight, oweight, var_,  o.var_);
              v_ = v_ + (oweight*var_)/(weight*(var_+o.var_))*(o.v_-v_);
              std::cout<<"after\n"<<v_<<"\n";
              if(!pcl_isfinite(v_.sum())) ROS_INFO("HERE");
              break;
            case DIRECTION:
              v_ = v_ + (oweight*var_)/(weight*(var_+o.var_))*(o.v_-v_);
              v_.normalize();
              break;

            default:
              ROS_ASSERT(0);
              break;
          }

          var_ = var_ - var_*var_/(var_+o.var_);
          std::cout<<"after\n"<<var_<<"\n";
        }
      }

    };

  private:

    S_ID ID_;
    bool is_plane;
    //Eigen::Vector3f nearest_point_;
    cob_3d_mapping_msgs::CurvedPolygon data_;
    Classification::Form form_;

    //from segmentation....
    Eigen::Matrix3f param_;
    Eigen::Matrix<float,3,2> proj2plane_;

    std::vector<S_FEATURE> features_;
    Outline outline_;
    std::vector<Eigen::Vector3f> points3d_;
    Eigen::Vector3f bb_min_, bb_max_;

    inline float modelAt(const float x, const float y)
    {
      return data_.parameter[0]+data_.parameter[1]*x+data_.parameter[2]*x*x+data_.parameter[3]*y+data_.parameter[4]*y*y+data_.parameter[5]*x*y;
    }

    void update() {
      ID_ += data_.ID;

      std::cout<<"ID: "<<data_.ID<<"\n";
      for(size_t i=0; i<data_.score.size(); i++)
      {
        std::cout<<data_.score[i].ID<<"\n";
        ID_ += data_.score[i].ID;
      }

      param_.col(0) = Eigen::Vector3f::Zero();
      param_.col(0)(2) = data_.parameter[0];

      param_.col(1)(0)=data_.parameter[1];
      param_.col(1)(1)=data_.parameter[3];
      param_.col(1)(2)=0;

      param_.col(2)(0)=data_.parameter[2];
      param_.col(2)(1)=data_.parameter[4];
      param_.col(2)(2)=data_.parameter[5];

      Eigen::Vector3f x,y;
      x(0)=y(1)=1.f;
      x(1)=x(2)=y(0)=y(2)=0.f;

      proj2plane_.col(0)=x;
      proj2plane_.col(1)=y;
      //      proj2plane_.col(0)=param_.col(1).cross(y);
      //      proj2plane_.col(1)=proj2plane_.col(0).cross(param_.col(1));
      //      proj2plane_.col(0).normalize();
      //      proj2plane_.col(1).normalize();
      //param_.col(1).cross(y).cross(param_.col(1));

      //      ROS_INFO("DOT %f %f %f %f %f", proj2plane_.col(0).dot(proj2plane_.col(1)), proj2plane_.col(0).dot(param_.col(1)), param_.col(1).dot(proj2plane_.col(1)), proj2plane_.col(0).norm(), proj2plane_.col(1).norm());

      //      param_.col(2)(0)*=proj2plane_(0,0)*proj2plane_(0,0);
      //      param_.col(2)(1)*=proj2plane_(1,1)*proj2plane_(1,1);
      //      param_.col(2)(2)*=proj2plane_(0,0)*proj2plane_(1,1);

      update_points3d();

      //calc nearest point
      Eigen::Vector3f nearest_point;
      nearest_point(0) = data_.features[2].x;
      nearest_point(1) = data_.features[2].y;
      nearest_point(2) = data_.features[2].z;

      if(param_.col(2).squaredNorm()>0.01f*0.01f)
      {
        is_plane = false;
        //TODO: calc point
      }
      else
      {
        is_plane = true;

        Eigen::Vector3f n,z=Eigen::Vector3f::Zero();
        z(2)=1;
        n=z;
        n(0)=-data_.parameter[1];
        n(1)=-data_.parameter[3];
        n.normalize();
        nearest_point = data_.parameter[0]*(z.dot(n))*n;
      }

      features_.clear();
      for(size_t i=0; i<data_.features.size(); i++)
      {
        if(data_.features[i].ID==1)
          features_.push_back( S_FEATURE(nearest_point,isPlane()));
        else
          features_.push_back( S_FEATURE(data_.features[i]));
      }

      int mains[4]={};
      Classification::Classification cl;
      Classification::QuadBB qbb;
      cl.setPoints(outline_.segments_);
      std::vector<Classification::Classification::EnergyPoint> compressed;
      form_.n_ = cl.buildLocalTree(false,&qbb,&compressed,NULL,mains);
      if(form_.n_)
        form_.n_->clean(1,form_.n_->energy_sum_*0.01f);

      for(int i=0; outline_.segments_.size()>3 && i<4; i++)
      {
        features_.push_back( S_FEATURE( points3d_[mains[i]] ));
        //        std::cout<<"MAIN\n"<<points3d_[mains[i]]<<"\n\n";
      }

      data_.polyline.clear();
      //      ROS_INFO("COMPRESSED %d", (int)compressed.size());
      for(size_t i=0; i<compressed.size(); i++)
      {
        cob_3d_mapping_msgs::polyline_point pt;
        pt.x = outline_.segments_[compressed[i].origin_](0);
        pt.y = outline_.segments_[compressed[i].origin_](1);
        pt.edge_prob = outline_.segments_[compressed[i].origin_](2);
        data_.polyline.push_back(pt);
      }
      update_points3d();

      char buf[128];
      static int cnt=0;
      sprintf(buf,"dbg%d.svg",cnt++);
      outline_.debug_svg(buf);

      //      std::cout<<"NP\n"<<nearest_point_<<"\n";
      //      std::cout<<"NP*\n"<<data_.features[0].x<<"\n"<<data_.features[0].y<<"\n"<<data_.features[0].z<<"\n";
    }

    void update_points3d()
    {
      points3d_.clear();
      outline_.segments_.clear();

      for(size_t i=0; i<data_.polyline.size(); i++)
      {
        Eigen::Vector2f pt;
        Eigen::Vector3f pt3;
        pt3(0) = pt(0) = data_.polyline[i].x;
        pt3(1) = pt(1) = data_.polyline[i].y;
        pt3(2) = data_.polyline[i].edge_prob;
        outline_.segments_.push_back( pt3 );
        points3d_.push_back( project2world(pt) );
        //        ROS_ERROR("%f %f",pt(0),pt(1));
        //        ROS_ERROR("%f %f %f %f %f %f",data_.parameter[0],data_.parameter[1],data_.parameter[2],data_.parameter[3],data_.parameter[4],data_.parameter[5]);
        //        ROS_ERROR("%f %f",points3d_.back()(2), modelAt(pt(0),pt(1)));
        //        ROS_ASSERT( std::abs(points3d_.back()(2)-modelAt(pt(0),pt(1))) < 0.01f );
      }

      update_BB();
    }

    void update_BB()
    {
      bb_min_.fill( std::numeric_limits<float>::max() );
      bb_max_.fill( std::numeric_limits<float>::min() );

      for(size_t i=0; i<points3d_.size(); i++)
      {
        bb_min_(0) = std::min(bb_min_(0), points3d_[i](0));
        bb_min_(1) = std::min(bb_min_(1), points3d_[i](1));
        bb_min_(2) = std::min(bb_min_(2), points3d_[i](2));

        bb_max_(0) = std::max(bb_max_(0), points3d_[i](0));
        bb_max_(1) = std::max(bb_max_(1), points3d_[i](1));
        bb_max_(2) = std::max(bb_max_(2), points3d_[i](2));
      }
    }

    Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const {
      Eigen::Vector3f pt2;
      pt2(0)=pt(0)*pt(0);
      pt2(1)=pt(1)*pt(1);
      pt2(2)=pt(0)*pt(1);

      return param_.col(0) + proj2plane_*pt + proj2plane_.col(0).cross(proj2plane_.col(1)) * (pt2.dot(param_.col(2)) + pt.dot(param_.col(1).head<2>()));
    }

  public:
    ex_curved_polygon(const cob_3d_mapping_msgs::CurvedPolygon& data):
      data_(data)//, form_(data.energy)
    {
      update();
    }

    virtual ~ex_curved_polygon()
    {
    }

    void getBB(Eigen::Vector3f &mi, Eigen::Vector3f &ma) const
    {
      mi = bb_min_;
      ma = bb_max_;
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
    {
      param_.col(0) = rot * param_.col(0)+tr;

      proj2plane_.col(0) = rot * proj2plane_.col(0);
      proj2plane_.col(1) = rot * proj2plane_.col(1);

#if 0
      {
        Eigen::Vector3f x,y,xy,t;
        x(0)=1.f;
        y(1)=1.f;
        x(1)=x(2)=y(0)=y(2)=0.f;
        xy=x+y;
        xy.normalize();

        param_.col(1)(0)=data_.parameter[1];
        param_.col(1)(1)=data_.parameter[3];
        param_.col(1)(2)=0;

        param_.col(2)(0)=data_.parameter[2];
        param_.col(2)(1)=data_.parameter[4];
        param_.col(2)(2)=data_.parameter[5];

        t(0)=(rot.transpose()*x).dot(param_.col(2));
        t(1)=(rot.transpose()*y).dot(param_.col(2));
        t(2)=0;
        t(2)=(rot.transpose()*xy).dot(param_.col(2)) - xy.dot(t);
        param_.col(2) = t;

        x = rot * x;
        y = rot * y;

        proj2plane_.col(0)=param_.col(1).cross(y);
        proj2plane_.col(1)=proj2plane_.col(0).cross(param_.col(1));
        proj2plane_.col(0).normalize();
        proj2plane_.col(1).normalize();
        //        proj2plane_.col(0)=param_.col(1).cross(y);
        //        proj2plane_.col(1)=param_.colf(1).cross(x);

        //        param_.col(2)(0)*=proj2plane_(0,0)*proj2plane_(0,0);
        //        param_.col(2)(1)*=proj2plane_(1,1)*proj2plane_(1,1);
        //        param_.col(2)(2)*=proj2plane_(0,0)*proj2plane_(1,1);
      }
#endif

      //TODO:
      //nearest_point_ = rot*nearest_point_;
      //      for(size_t i=0; i<points3d_.size(); i++)
      //      {
      //        points3d_[i] = rot*points3d_[i];
      //      }

      //TODO:
      //if(isPlane())
      //{
      //  if(tr.squaredNorm()) nearest_point_ += nearest_point_*(tr.dot(nearest_point_))/(nearest_point_.squaredNorm()); //TODO: optimize
      //}
      //else
      //  nearest_point_ += tr;

      for(size_t i=0; i<features_.size(); i++)
        features_[i].transform(rot,tr,var_R,var_T);

      for(size_t i=0; i<points3d_.size(); i++)
      {
        points3d_[i] = rot*points3d_[i]+tr;
      }

      update_BB();
      //      bb_min_ = rot*bb_min_;
      //      bb_max_ = rot*bb_max_;
      //      bb_min_+=tr;
      //      bb_max_+=tr;
    }

    inline const S_ID &getID() const {return ID_;}

    bool isPlane() const {return is_plane;}

    Eigen::Vector3f getNearestPoint() const {return features_[0].v_;}
    Eigen::Vector3f getNearestTransformedPoint(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const
    {
      S_FEATURE temp = features_[0];
      temp.transform(rot,tr,0,0);
      return temp.v_;
    }

    const std::vector<cob_3d_mapping_msgs::simalarity_score> &getScore() const {return data_.score;}

    //TODO: change to internal
    const std::vector<S_FEATURE> &getFeatures() const {return features_;}

    /**
     * checks intersection of two bounding boxes
     *
     * "add" adds value, so flat planes can intersect too
     */
    bool intersectsBB(const ex_curved_polygon &o, const float add=0) const
    {
      //      std::cout<<"min\n"<<bb_min_<<"\n";
      //      std::cout<<"max\n"<<bb_max_<<"\n";
      //      std::cout<<"min\n"<<o.bb_min_<<"\n";
      //      std::cout<<"max\n"<<o.bb_max_<<"\n\n";
      return
          bb_min_(0)<=o.bb_max_(0)+add && bb_max_(0)+add>=o.bb_min_(0) &&
          bb_min_(1)<=o.bb_max_(1)+add && bb_max_(1)+add>=o.bb_min_(1) &&
          bb_min_(2)<=o.bb_max_(2)+add && bb_max_(2)+add>=o.bb_min_(2) ;
    }

    bool fitsCurvature(const ex_curved_polygon &o, const float thr) const
    {
      //      if( param_.col(2).squaredNorm()<0.1 && o.param_.col(2).squaredNorm()<0.1)
      //        return true;
      return param_.col(2).cross(o.param_.col(2)).squaredNorm() < thr*thr;
    }

    //TODO: do it more accurately? but its fast
    bool extensionMatch(const ex_curved_polygon &o, const float thr, const float var) const
    {
      const float l1 = (bb_max_-bb_min_).norm();
      const float l2 = (o.bb_max_-o.bb_min_).norm();

      return std::max(std::abs(l1-l2)-var,0.f) < std::min(l1,l2) * thr;
    }

    float bestMatch(const ex_curved_polygon &o, std::map<size_t,size_t> &r, std::vector<size_t> &m1, std::vector<size_t> &m2, float e=0, float e_min=std::numeric_limits<float>::max())
    {
      if(m2.size()==0 || m1.size()==0 || e>e_min) return e;

      for(size_t i=0; i<m1.size(); i++)
      {
        size_t t1 = m1[i];
        m1.erase(m1.begin()+i);

        for(size_t j=0; j<m2.size(); j++)
        {
          size_t t2 = m2[j];
          m2.erase(m2.begin()+j);

          float er = bestMatch(o,r,m1,m2,e+(features_[t1].v_-o.features_[t2].v_).norm(), e_min);

          if(er < e_min)
          {
            e_min = er;
            r[t1] = t2;
          }

          m2.insert(m2.begin()+j,t2);
        }

        m1.insert(m1.begin()+i,t1);
      }

      return e_min;
    }

    bool merge(const ex_curved_polygon &o1, const ex_curved_polygon &o2) {
      //      PolygonData p1, p2, p3;
      //
      //      for(size_t i=0; i<o1.outline_.segments_.size(); i++)
      //      {
      //        //if(o1.segments_[i](2)>0.5f)
      //        {
      //          Eigen::Vector2f p = nextPoint(o1.project2world(o1.outline_.segments_[i].head<2>() ));
      //          p1.add(p(0), p(1));
      //          //          std::cout<<"p1\n"<<p<<"\n";
      //          //          std::cout<<"s1\n"<<o1.segments_[i]<<"\n";
      //          //          std::cout<<"w1\n"<<project2world(p )<<"\n";
      //        }
      //      }
      //      std::cout<<"PARAMS\n"<<proj2plane_<<"\n";
      //      std::cout<<param_<<"\n";
      //      for(size_t i=0; i<o2.outline_.segments_.size(); i++)
      //      {
      //        //if(o2.segments_[i](2)>0.5f)
      //        {
      //          Eigen::Vector2f p = nextPoint(o2.project2world(o2.outline_.segments_[i].head<2>() ));
      //          p2.add(p(0), p(1));
      //          //          std::cout<<"p2\n"<<p<<"\n";
      //          //          std::cout<<"s2\n"<<o2.segments_[i]<<"\n";
      //          //          std::cout<<"w2\n"<<project2world(p )<<"\n";
      //        }
      //      }
      //
      //      mergePolygons(p1,p2, p3);
      //
      //      if(p3.get().size()<=0)
      //        return false;

      Outline p1, p2;

      p1.weight_ = o1.data_.weight;
      p2.weight_ = o2.data_.weight;

      for(size_t i=0; i<o1.outline_.segments_.size(); i++)
      {
        //if(o1.segments_[i](2)>0.5f)
        {
          Eigen::Vector3f p;
          p.head<2>() = nextPoint(o1.project2world(o1.outline_.segments_[i].head<2>() ));
          p(2) = o1.data_.weight*o1.outline_.segments_[i](2);
          p1 += p;

          ROS_INFO("dist %f", (o1.project2world(o1.outline_.segments_[i].head<2>())-project2world(p.head<2>())).norm() );
        }
      }

      for(size_t i=0; i<o2.outline_.segments_.size(); i++)
      {
        //if(o2.segments_[i](2)>0.5f)
        {
          Eigen::Vector3f p;
          p.head<2>() = nextPoint(o2.project2world(o2.outline_.segments_[i].head<2>() ));
          p(2) = o2.data_.weight*o2.outline_.segments_[i](2);
          p2 += p;

          ROS_INFO("dist %f", (o2.project2world(o2.outline_.segments_[i].head<2>())-project2world(p.head<2>())).norm() );
        }
      }

      static int cnt=0;
      ++cnt;
      char buffer[128];
      //      sprintf(buffer,"%dA.svg",cnt);
      //      p1.debug_svg(buffer);
      //      sprintf(buffer,"%dB.svg",cnt);
      //      p2.debug_svg(buffer);

      p1.reverse(); //TODO: change alog. instead of this
      p2.reverse();
      outline_ = p1+p2;
      outline_.reverse();

//      for(size_t i=0; i<outline_.segments_.size(); i++)
//        outline_.segments_[i](2) /= std::max(o1.data_.weight, o2.data_.weight);

      sprintf(buffer,"%f %dC.svg",Debug::Interface::get().getTime(), cnt);
      p1.debug_svg(p2.debug_svg(outline_.debug_svg(buffer,200,500,"red"),200,500,"green"),200,500);

      std::vector<Classification::Classification::EnergyPoint> compressed;
      int mains[4]={};
      Classification::Classification cl;
      Classification::QuadBB qbb;
      cl.setPoints(outline_.segments_);
      form_.n_ = cl.buildLocalTree(false,&qbb,&compressed,NULL,mains);
      if(form_.n_)
        form_.n_->clean(1,form_.n_->energy_sum_*0.01f);

      for(size_t i=0; i<features_.size(); i++)
      {
        if(features_[i].ID_==-1)
        {
          features_.erase(features_.begin()+i);
          --i;
        }
      }

      for(int i=0; outline_.segments_.size()>3 && i<4; i++)
      {
        features_.push_back( S_FEATURE( project2world(outline_.segments_[mains[i]].head<2>()) ));
        //        std::cout<<"MAIN\n"<<project2world(outline_.segments_[mains[i]].head<2>())<<"\n\n";
      }

      data_.polyline.clear();

      for(size_t i=0; i<compressed.size(); i++)
      {
        cob_3d_mapping_msgs::polyline_point pt;
        pt.x = outline_.segments_[compressed[i].origin_](0);
        pt.y = outline_.segments_[compressed[i].origin_](1);
        pt.edge_prob = outline_.segments_[compressed[i].origin_](2);
        data_.polyline.push_back(pt);
      }
      //      for(size_t i=0; i<outline_.segments_.size(); i++)
      //      {
      //        //        Eigen::Vector2f v1;
      //        ::cob_3d_mapping_msgs::polyline_point pt;
      //        //        v1(0)=p3.get()[i*2+0];
      //        //        v1(1)=p3.get()[i*2+1];
      //        //#error teile durch 2
      //
      //        //        std::cout<<"p3\n"<<v1<<"\n";
      //
      //        pt.x = outline_.segments_[i](0);
      //        pt.y = outline_.segments_[i](1);
      //        pt.edge_prob = outline_.segments_[i](2);
      //
      //        data_.polyline.push_back(pt);
      //      }

      update_points3d();

      return true;
    }

    /**
     * update parameters, ... from other obj. (TODO:)
     */
    bool operator+=(const ex_curved_polygon &o) {
      //merge parameters to third manifold
      ex_curved_polygon o2 = *this;
      //o2.proj2plane_
      //TODO:
      if(o.outline_.segments_.size()<1 || outline_.segments_.size()<1)
        return false;

      if(!canMerge(o))
        return false;

      if(1){
        param_.col(0) = (param_.col(0)*data_.weight + o.param_.col(0)*o.data_.weight)/(data_.weight+o.data_.weight);
        param_.col(1) = (param_.col(1)*data_.weight + o.param_.col(1)*o.data_.weight)/(data_.weight+o.data_.weight);
        param_.col(1).normalize();

        const float xx = proj2plane_.col(0).dot(o.proj2plane_.col(0));
        const float xy = proj2plane_.col(0).dot(o.proj2plane_.col(1));
        data_.parameter[1] = xx*o.param_.col(1)(0) + xy*o.param_.col(1)(1);

        const float yx = proj2plane_.col(1).dot(o.proj2plane_.col(0));
        const float yy = proj2plane_.col(1).dot(o.proj2plane_.col(1));
        data_.parameter[3] = yx*o.param_.col(1)(0) + yy*o.param_.col(1)(1);

        data_.parameter[2] = xx*xx*o.param_.col(2)(0) + xy*xy*o.param_.col(2)(1) + xx*xy*o.param_.col(2)(2);
        data_.parameter[4] = yx*yx*o.param_.col(2)(0) + yy*yy*o.param_.col(2)(1) + yx*yy*o.param_.col(2)(2);

        Eigen::Vector3f bt = proj2plane_.col(0)+proj2plane_.col(1);
        bt.normalize();
        const float bx = bt.dot(o.proj2plane_.col(0));
        const float by = bt.dot(o.proj2plane_.col(1));

        data_.parameter[5] = bx*bx*o.param_.col(2)(0) + by*by*o.param_.col(2)(1) + bx*by*o.param_.col(2)(2)
                - (bx*bx*data_.parameter[2] + by*by*data_.parameter[4]);

        for(int i=0; i<6; i++)
          data_.parameter[i] = (data_.parameter[i]*o.data_.weight + o2.data_.parameter[i]*data_.weight)/(data_.weight+o.data_.weight);

        param_.col(1)(0)=data_.parameter[1];
        param_.col(1)(1)=data_.parameter[3];
        param_.col(1)(2)=0;

        param_.col(2)(0)=data_.parameter[2];
        param_.col(2)(1)=data_.parameter[4];
        param_.col(2)(2)=data_.parameter[5];
      }

      //combine outline
      if(!merge(o,o2))
      {
        param_ = o2.param_;
        proj2plane_ = o2.proj2plane_;
        data_.parameter = o2.data_.parameter;
        ROS_INFO("could not merge");
        return false;
      }

      ID_ += o.ID_;

      std::map<size_t,size_t> cor;
      std::vector<size_t> matches1,matches2;
      for(size_t i=0; i<std::min(features_.size(),o.features_.size()); i++)
      {
        if(features_[i].ID_!=-1) features_[i].merge(o.features_[i],data_.weight,o.data_.weight);
        else matches1.push_back(i);
      }

      //      matches2=matches1;
      //      float m=bestMatch(o,cor,matches1,matches2);
      //      ROS_INFO("dist bestMatch %f",m);
      //      ROS_INFO("cor.size() %d",cor.size());
      //      std::cout.flush();
      //
      //      ROS_ASSERT(cor.size()==4);
      //      if(m<0.08f*4)
      //      {
      //        for(std::map<size_t,size_t>::const_iterator it = cor.begin(); it!=cor.end(); it++)
      //        {
      //          ROS_INFO("cor %d %d",it->first,it->second);
      //          std::cout<<features_[it->first].v_<<"-\n"<<o.features_[it->second].v_<<"\n";
      //          features_[it->first].merge(o.features_[it->second]);
      //        }
      //
      //        if( !(form_ += o.form_) )
      //        {
      //          matchForm(o);
      //          //debug_form();
      //          //o.debug_form();
      //        }
      //      }

      data_.weight += o.data_.weight;

      return true;
    }

    void debug_form() const
    {
      Classification::Classification cl;
      Classification::QuadBB qbb;
      cl.setPoints(outline_.segments_);
      cl.buildLocalTree(true,&qbb);
    }

    bool matchForm(const ex_curved_polygon &o) const {
      if(!form_.n_ || ! o.form_.n_)
        return false;

      float e = form_.n_->search(*o.form_.n_,0.8);

      //if(e>std::max(form_.n_->energy_sum_, o.form_.n_->energy_sum_))
      //      if(e<0.4*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_)&&std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_)>1)
      //      {
      //        debug_form();
      //        o.debug_form();
      //
      //        form_.n_->print();
      //        o.form_.n_->print();
      //        ROS_INFO("could");
      //      }

      ROS_INFO("match %f %f",e,0.2f*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_));
      return e<0.4f*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_);
    }

    float matchFormf(const ex_curved_polygon &o) const {
      if(!form_.n_ || ! o.form_.n_)
        return false;

      return std::max(0.1f,1.1f-form_.n_->search(*o.form_.n_,0.8)/std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_));
    }

    float getEnergy() const {return form_.n_->energy_;}
    float getWeight() const {return data_.weight;}
    void printEnergy() const {form_.n_->print();}

    void getTriangles(std::vector<Eigen::Vector3f> &tri) const
    {
      for(size_t i=0; i<features_.size(); i++)
      {
        if(features_[i].ID_!=-1 || !pcl_isfinite(features_[2].v_.sum()) || !pcl_isfinite(features_[i].v_.sum())) continue;
        if(tri.size()>2)
          tri.push_back( tri[tri.size()-1] );
        else
          tri.push_back( features_[i].v_ );
        tri.push_back( features_[2].v_ );
        tri.push_back( features_[i].v_ );
      }
      if(tri.size()>2)
        tri[0] = tri[tri.size()-1];
    }

    struct MyFunctor
    {
      const float a,b,c, d,e;
      const float x0,y0,z0;

      int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
      {
        // distance
        fvec(0) = std::pow((-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)+d*x(0)+e*x(1)),2)+std::pow(x(1)-y0,2)+std::pow(x(0)-x0,2);
        fvec(1) = 0;

        //        std::cout<<"fvec\n"<<fvec<<std::endl;
        return 0;
      }

      int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
      {
        //Jacobian
        //TODO:
        fjac(0,0) = 2*(c*x(1)+2*a*x(0)+d)*(-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)+d*x(0)+e*x(1))+2*(x(0)-x0);
        fjac(0,1) = 2*(2*b*x(1)+c*x(0)+e)*(-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)+d*x(0)+e*x(1))+2*(x(1)-y0);
        fjac(1,0) = fjac(1,1) = 0;
        return 0;
      }

      int inputs() const { return 2; }
      int values() const { return 2; } // number of constraints
    };

    /// find nearest point to manifold (Newton)
    Eigen::Vector2f _nextPoint(const Eigen::Vector3f &v, Eigen::Vector3f p, const int depth=0) const
    {
      Eigen::VectorXf r(2);
      r = v.head<2>();
      MyFunctor functor={param_.col(2)(0),param_.col(2)(1),param_.col(2)(2),
                         param_.col(1)(0),param_.col(1)(1),
                         p(0),p(1),p(2)};
      Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);
      lm.parameters.maxfev = 50;
      lm.minimize(r);

      return r;
#if 0
if(depth>10)
{
  ROS_INFO("break as depth");
  return p.head<2>();
}

//            std::cout<<"p\n"<<p<<"\n";
p(2) = param_.col(2)(0)*p(0)*p(0)+param_.col(2)(1)*p(1)*p(1)+param_.col(2)(2)*p(0)*p(1);
//            std::cout<<"p\n"<<p<<"\n";

Eigen::Vector3f n;
n(0) = -(param_.col(2)(0)*2*p(0)+param_.col(2)(2)*p(1));
n(1) = -(param_.col(2)(1)*2*p(1)+param_.col(2)(2)*p(0));
n(2) = 1;

Eigen::Vector3f d = ((p-v).dot(n)/n.squaredNorm()*n+(v-p));

//            std::cout<<"d\n"<<d<<"\n";
//            std::cout<<"n\n"<<n<<"\n";
//            std::cout<<"pv\n"<<(p-v)/(p-v)(2)<<"\n";
//            std::cout<<"pv\n"<<(p-v)<<"\n";
//            std::cout<<"dd\n"<<(p-v).dot(n)/n.squaredNorm()*n<<"\n";
//ROS_ASSERT(std::abs(d(2))<=std::abs(z));

if(!pcl_isfinite(d.sum()) || d.head<2>().squaredNorm()<0.001f*0.001f)
{
  //                std::cout<<"---------------\n";
  return (p+d).head<2>();
}
return _nextPoint(v,p+d,depth+1);
#endif
    }

    /// find nearest point to manifold (Newton)
    Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const
    {
      //      Eigen::Matrix3f M;
      //      M.col(0) = proj2plane_.col(0);
      //      M.col(1) = proj2plane_.col(1);
      //      M.col(2) = param_.col(2);
      //M = M.inverse().eval();

      //Eigen::Vector2f p = (M*(v-param_.col(0))).head<2>();
      Eigen::Vector3f p;

      //      std::cout<<"v\n"<<v<<"\n";
      //      std::cout<<"param_.col(0)\n"<<param_.col(0)<<"\n";
      //      std::cout<<"proj2plane_\n"<<proj2plane_<<"\n";

      p(0) = (v-param_.col(0)).dot(proj2plane_.col(0))/proj2plane_.col(0).squaredNorm();
      p(1) = (v-param_.col(0)).dot(proj2plane_.col(1))/proj2plane_.col(1).squaredNorm();
      p(2) = (v-param_.col(0)).dot(proj2plane_.col(0).cross(proj2plane_.col(1)));

      Eigen::Vector2f r = _nextPoint(p, p);

      float e1 = (v-project2world(p.head<2>())).norm();
      float e2 = (v-project2world(r)).norm();
      //      std::cout<<"dist: "<<e1<<"  "<<e2<<"\n";
      //      std::cout.flush();

      //ROS_ASSERT(e1+0.1f>=e2 || !pcl_isfinite(e1));

      if(e1+0.001f<e2)
      {
        ROS_WARN("e1>=e2 %f %f",e1,e2);
        return p.head<2>();
      }

      return r;
    }

    bool canMerge(const ex_curved_polygon &o) const {
      PolygonData p1, p2, p3;

      for(size_t i=0; i<o.outline_.segments_.size(); i++)
      {
        Eigen::Vector2f p = nextPoint(o.project2world(o.outline_.segments_[i].head<2>() ));
        p1.add(
            p(0),p(1)
            //            ((o.segments_[i](0) * o.proj2plane_(0,0)+o.param_.col(0)(0))-param_.col(0)(0))/proj2plane_(0,0),
            //            ((o.segments_[i](1) * o.proj2plane_(1,1)+o.param_.col(0)(1))-param_.col(0)(1))/proj2plane_(1,1)
        );
      }
      for(size_t i=0; i<outline_.segments_.size(); i++)
      {
        p2.add(outline_.segments_[i](0),outline_.segments_[i](1));
      }

      //      std::cout<<"off1\n"<<param_.col(0)<<"\n";
      //      std::cout<<"off2\n"<<o.param_.col(0)<<"\n";
      //      std::cout<<"n1\n"<<param_.col(1)<<"\n";
      //      std::cout<<"n2\n"<<o.param_.col(1)<<"\n";
      //      std::cout<<"p1\n"<<param_.col(2)<<"\n";
      //      std::cout<<"p2\n"<<o.param_.col(2)<<"\n";

      if(unionPolygons(p1,p2, p3)<0.4f)
      {
//        ROS_INFO("merge break 1");
        return false;
      }

      float lges=0.f, er=0.f, erlast=0.f, lfirst=0.f;
      float er2=0;

      for(size_t i=0; i<p3.get().size()/2; i++)
      {
        Eigen::Vector2f v1, v3;
        Eigen::Vector3f t, n, v2;
        v1(0)=p3.get()[i*2+0];
        v1(1)=p3.get()[i*2+1];
        //        v2(0) = ((v1(0)*proj2plane_(0,0)+param_.col(0)(0))-o.param_.col(0)(0))/o.proj2plane_(0,0);
        //        v2(1) = ((v1(1)*proj2plane_(1,1)+param_.col(0)(1))-o.param_.col(0)(1))/o.proj2plane_(1,1);

        t = project2world(v1);
        v2 = o.project2world(o.nextPoint(t));

        v1(0)=p3.get()[(i*2+2)%p3.get().size()];
        v1(1)=p3.get()[(i*2+3)%p3.get().size()];
        n = project2world(v1);

        const float l = (t-n).norm();
        er += l*erlast;
        erlast = (t-v2).norm();
        er2 += erlast;
        er += l*erlast;
        lges += 2*l;

        //        std::cout<<"coord\t\t"<<erlast<<"  "<<l<<"\n"<<t<<"\n\n"<<v2<<"\n\n";

        if(i==0) lfirst = l;

      }
      //      Outline p3 = outline_|o.outline_;
      //
      //      float lges=0.f, er=0.f, erlast=0.f, lfirst=0.f;
      //      float er2=0;
      //
      //      for(size_t i=0; i<p3.segments_.size(); i++)
      //      {
      //        Eigen::Vector2f v1=p3.segments_[i].head<2>(), v3;
      //        Eigen::Vector3f t, n, v2;
      //        //        v2(0) = ((v1(0)*proj2plane_(0,0)+param_.col(0)(0))-o.param_.col(0)(0))/o.proj2plane_(0,0);
      //        //        v2(1) = ((v1(1)*proj2plane_(1,1)+param_.col(0)(1))-o.param_.col(0)(1))/o.proj2plane_(1,1);
      //
      //        t = project2world(v1);
      //        v2 = o.project2world(o.nextPoint(t));
      //
      //        v1=p3.segments_[(i+1)%p3.segments_.size()].head<2>();
      //        n = project2world(v1);
      //
      //        const float l = (t-n).norm();
      //        er += l*erlast;
      //        erlast = (t-v2).norm();
      //        er2 += erlast;
      //        er += l*erlast;
      //        lges += 2*l;
      //
      //        //        std::cout<<"coord\t\t"<<erlast<<"  "<<l<<"\n"<<t<<"\n\n"<<v2<<"\n\n";
      //
      //        if(i==0) lfirst = l;
      //
      //      }

      er += lfirst*erlast;

      if(isPlane()) ROS_INFO("PLANE");
      ROS_INFO("proj %f", er/lges);

      return er/lges<0.05f;
    }

    bool invalid() const
    {
      if(!pcl_isfinite(getNearestPoint().sum())) {
        ROS_ERROR("rm3");
        return true;
      }

      if(!pcl_isfinite(features_[2].v_.sum()) || features_[2].v_(2)<0.f)
      {
        ROS_ERROR("rm1");
        return true;
      }

      for(size_t i=0; i<points3d_.size(); i++)
      {
        if(!pcl_isfinite(points3d_[i].sum()) || points3d_[i](2)<0.f)
        {
          ROS_ERROR("rm2");
          return true;
        }
      }

      return false;
    }
  };

}


#endif /* OBJECT_H_ */
