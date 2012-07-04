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

      S_FEATURE(const cob_3d_mapping_msgs::feature &ft):var_(0.1f) //TODO: default value?
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
        v_(v), var_(0.1f) //TODO: default value?
      {
        type_ = bPlane?NORMAL:POINT;
        ID_ = 1;
      }

      S_FEATURE(const Eigen::Vector3f &v):
        v_(v), var_(0.1f) //TODO: default value?
      {
        type_ = POINT;
        ID_ = -1;
      }

      void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
      {
        switch(type_)
        {
          case POINT:
            var_ = v_.norm()*var_R+var_T;
            v_ = rot*v_ + tr;
            break;
          case NORMAL:
            var_ = v_.norm()*var_R+var_T; //TODO: should be middle not v
            v_ = rot*v_;
            if(tr.squaredNorm())
              v_ += v_*(tr.dot(v_))/(v_.squaredNorm()); //TODO: optimize
            break;
          case DIRECTION:
            v_ = rot*v_;
            var_ = var_R;
            break;
          default:
            ROS_ASSERT(0);
            break;
        }
      }

      void merge(const S_FEATURE &o)
      {
        if(type_ != o.type_)
        {
          ROS_INFO("cannot merge features of differnt type");
          return;
        }

        switch(type_)
        {
          case POINT:
          case NORMAL:
            v_ = v_ + var_/(var_+o.var_)*(o.v_-v_);
            break;
          case DIRECTION:
            v_ = v_ + var_/(var_+o.var_)*(o.v_-v_);
            v_.normalize();
            break;

          default:
            ROS_ASSERT(0);
            break;
        }

        var_ = var_ - var_*var_/(var_+o.var_);
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
    std::vector<Eigen::Vector3f> segments_;
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

      if(std::abs(data_.parameter[5])<0.0001f) {
        param_.col(0)(0)=-data_.parameter[1]/(2*data_.parameter[2]);
        param_.col(0)(1)=-data_.parameter[3]/(2*data_.parameter[4]);
      }
      else {
        param_.col(0)(1)=
            ((2*data_.parameter[2]*data_.parameter[3])/data_.parameter[5]-data_.parameter[1]) /
            ( data_.parameter[5]-(4*data_.parameter[2]*data_.parameter[4]/data_.parameter[5]));

        param_.col(0)(0)=-(data_.parameter[1]+data_.parameter[5]*param_.col(0)(1))/(2*data_.parameter[2]);
      }

      if(!pcl_isfinite(param_.col(0)(0)))
        param_.col(0)(0)=0.f;
      if(!pcl_isfinite(param_.col(0)(1)))
        param_.col(0)(1)=0.f;


      param_.col(0)(2)=modelAt(param_.col(0)(0), param_.col(0)(1));

      param_.col(1)(0)=-(data_.parameter[1]+2*data_.parameter[2]*param_.col(0)(0)+data_.parameter[5]*param_.col(0)(1));
      param_.col(1)(1)=-(data_.parameter[3]+2*data_.parameter[4]*param_.col(0)(1)+data_.parameter[5]*param_.col(0)(0));
      param_.col(1)(2)=1;
      param_.col(1).normalize();

      param_.col(2)(0)=data_.parameter[2];
      param_.col(2)(1)=data_.parameter[4];
      param_.col(2)(2)=data_.parameter[5];

      Eigen::Vector3f x,y;
      x(0)=1.f;
      y(1)=1.f;
      x(1)=x(2)=y(0)=y(2)=0.f;

      proj2plane_.col(0)=param_.col(1).cross(y);
      proj2plane_.col(1)=param_.col(1).cross(x);

      param_.col(2)(0)*=proj2plane_(0,0)*proj2plane_(0,0);
      param_.col(2)(1)*=proj2plane_(1,1)*proj2plane_(1,1);
      param_.col(2)(2)*=proj2plane_(0,0)*proj2plane_(1,1);

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
      cl.setPoints(segments_);
      form_.n_ = cl.buildLocalTree(false,&qbb,NULL,NULL,mains);
      form_.n_->clean(1,form_.n_->energy_sum_*0.01f);

      for(int i=0; segments_.size()>3 && i<4; i++)
      {
        features_.push_back( S_FEATURE( points3d_[mains[i]] ));
      }

      //      std::cout<<"NP\n"<<nearest_point_<<"\n";
      //      std::cout<<"NP*\n"<<data_.features[0].x<<"\n"<<data_.features[0].y<<"\n"<<data_.features[0].z<<"\n";
    }

    void update_points3d()
    {
      points3d_.clear();

      for(size_t i=0; i<data_.polyline.size(); i++)
      {
        Eigen::Vector2f pt;
        Eigen::Vector3f pt3;
        pt3(0) = pt(0) = data_.polyline[i].x;
        pt3(1) = pt(1) = data_.polyline[i].y;
        pt3(2) = 0.f;
        segments_.push_back( pt3 );
        points3d_.push_back( project2world(pt) );
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

    Eigen::Vector3f project2world(const Eigen::Vector2f &pt) {
      Eigen::Vector3f pt2;
      pt2(0)=pt(0)*pt(0);
      pt2(1)=pt(1)*pt(1);
      pt2(2)=pt(0)*pt(1);

      return param_.col(0) + proj2plane_*pt + param_.col(1)*(pt2).dot(param_.col(2));
    }

  public:
    ex_curved_polygon(const cob_3d_mapping_msgs::CurvedPolygon& data):
      data_(data), form_(data.energy)
    {
      update();
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
    {
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
        points3d_[i] += tr;
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
    bool extensionMatch(const ex_curved_polygon &o, const float thr) const
    {
      const float l1 = (bb_max_-bb_min_).norm();
      const float l2 = (o.bb_max_-o.bb_min_).norm();

      return std::abs(l1-l2) < std::min(l1,l2) * thr;
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

    /**
     * update parameters, ... from other obj. (TODO:)
     */
    void operator+=(const ex_curved_polygon &o) {
      ID_ += o.ID_;
      ROS_ASSERT(features_.size()==o.features_.size());

      std::map<size_t,size_t> cor;
      std::vector<size_t> matches1,matches2;
      for(size_t i=0; i<features_.size(); i++)
      {
        if(features_[i].ID_!=-1) features_[i].merge(o.features_[i]);
        else matches1.push_back(i);
      }

      matches2=matches1;
      float m=bestMatch(o,cor,matches1,matches2);
      ROS_INFO("dist bestMatch %f",m);

      ROS_ASSERT(cor.size()==4);
      if(m<0.08f*4)
      {
        for(std::map<size_t,size_t>::const_iterator it = cor.begin(); it!=cor.end(); it++)
        {
          ROS_INFO("cor %d %d",it->first,it->second);
          std::cout<<features_[it->first].v_<<"-\n"<<o.features_[it->second].v_<<"\n";
          features_[it->first].merge(o.features_[it->second]);
        }

        if( !(form_ += o.form_) )
        {
          matchForm(o);
          //debug_form();
          //o.debug_form();
        }
      }
    }

    void debug_form() const
    {
      Classification::Classification cl;
      Classification::QuadBB qbb;
      cl.setPoints(segments_);
      cl.buildLocalTree(true,&qbb);
    }

    bool matchForm(const ex_curved_polygon &o) const {
      float e = form_.n_->search(*o.form_.n_,0.8);

      //if(e>std::max(form_.n_->energy_sum_, o.form_.n_->energy_sum_))
      if(e<0.4*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_)&&std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_)>1)
      {
        debug_form();
        o.debug_form();

        form_.n_->print();
        o.form_.n_->print();
        ROS_INFO("could");
      }

      ROS_INFO("match %f %f",e,0.2f*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_));
      return e<0.4f*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_);
    }

    float matchFormf(const ex_curved_polygon &o) const {
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

  };

}


#endif /* OBJECT_H_ */
