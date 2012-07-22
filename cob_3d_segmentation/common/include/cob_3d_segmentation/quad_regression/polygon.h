/*
 * polygon.h
 *
 *  Created on: 14.06.2012
 *      Author: josh
 */

#ifndef POLYGON_H_
#define POLYGON_H_

#include "sub_structures.h"
#include <cob_3d_mapping_msgs/CurvedPolygon.h>

namespace Segmentation
{

#ifdef USE_BOOST_POLYGONS_
#include <boost/polygon/polygon.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
  using namespace boost::polygon::operators;

  typedef boost::polygon::polygon_with_holes_data<int> BoostPolygon;
  typedef BoostPolygon::point_type BoostPoint;
  typedef std::vector<BoostPolygon> BoostPolygonSet;
  typedef boost::polygon::polygon_90_set_traits<BoostPolygonSet> BoostTraits;
#endif

  struct S_POLYGON {
    typedef Eigen::Matrix<float, 6, 1>    Vector6f;

    Eigen::Matrix3f param_;
    std::vector<std::vector<Eigen::Vector3f> >  segments_;
#ifdef USE_BOOST_POLYGONS_
    std::vector<std::vector<BoostPoint> >       segments2d_;
#endif
    float weight_; //number of points used for this model
#ifdef CHECK_CONNECTIVITY
    S_POLYGON_CONNECTIVITY connectivity_;
#endif
    SubStructure::Model model_;
    Eigen::Vector3f middle_;

    Eigen::Matrix<float,3,2> proj2plane_;
    Vector6f param_xy_;

#ifdef USE_CLASSIFICATION_
    Classification::Form::Ptr form_;
#endif

    Eigen::Vector3f feature_;

    S_POLYGON()
#ifdef USE_CLASSIFICATION_
    :form_(new Classification::Form)
#endif
    {}

    bool isLinear() const
    {
      std::cout<<model_.p<<"\n";
      return std::abs(model_.p(2))<0.01f && std::abs(model_.p(4))<0.01f && std::abs(model_.p(5))<0.01f;
    }

    void buildForm(const bool debug=false) {
      if(segments_.size()<1)
        return;

      std::vector<Eigen::Vector3f> pts;
      for(size_t i=0; i<segments_[0].size(); i++)
      {
        Eigen::Vector3f v;
#ifdef USE_BOOST_POLYGONS_
        v(0)=segments2d_[0][i].x();
        v(1)=segments2d_[0][i].y();
#endif
        pts.push_back(v);
        //ROS_ERROR("%f %f",segments_[0][i](0),segments_[0][i](1));
      }

#ifdef USE_CLASSIFICATION_
      Classification::Classification cl;
      Classification::QuadBB qbb;

      form_->curvature_ = getInvariantCurvature();
      cl.setPoints(segments_[0]);
      //cl.setPoints(pts);

      int mains[2];
      form_->n_=cl.buildLocalTree(debug,&qbb,&form_->compressed_energy_,mains);
      form_->outline_ = qbb.getOutline();

      feature_ = project2world(segments_[0][mains[0]].head<2>());
      feature_-= project2world(segments_[0][mains[1]].head<2>());
      //feature_.normalize();

      if(feature_(0)<0) feature_=-feature_;
#endif
    }

    Eigen::Vector2f project2plane(const float x, const float y, const SubStructure::Model &model) {
      Eigen::Vector2f mv;
      mv(0)=x;
      mv(1)=y;
      float mz;
      mz=model.intersect(mv(0), mv(1));
      //mv=mv*mz-getCenter();
      Eigen::Vector3f v;
      v.head<2>() = mz * mv;
      v(2) = mz;
      mv(0) = (v-param_.col(0)).dot(proj2plane_.col(0))/proj2plane_.col(0).squaredNorm();
      mv(1) = (v-param_.col(0)).dot(proj2plane_.col(1))/proj2plane_.col(1).squaredNorm();
#if DEBUG_LEVEL >70
      if(!pcl_isfinite(mv(0))) {
        ROS_ERROR("mist");
        std::cerr<<x<<","<<y<<","<<mz<<"\nmv:\n";
        std::cerr<<mv<<"\np\n";
        std::cerr<<model.p<<"\n";
        std::cerr<<"mz: "<<mz<<"\nmv:\n";
        //getchar();
      }
#endif
      return mv;
    }

#ifdef USE_BOOST_POLYGONS_
    BoostPolygonSet getBoostPolygon2D() const {
      //TODO: holes
      BoostPolygon poly;
      BoostPolygonSet r;
      if(segments2d_.size()==0) return r;
      poly.set(segments2d_[0].begin(), segments2d_[0].end());
      //poly.set_holes(segments2d_.begin(), segments2d_.end());
      r.push_back(poly);
      return r;
    }

    BoostPolygonSet getBoostPolygon2D_Centered() const {
      //TODO: holes
      BoostPolygon poly;
      BoostPolygonSet r;
      if(segments2d_.size()==0) return r;
      {
        poly.set(segments2d_[0].begin(), segments2d_[0].end());
        //poly.set_holes(segments2d_.begin(), segments2d_.end());

        BoostPoint cnt;
        boost::polygon::center(cnt,poly);
        std::vector<BoostPoint> temp;

        for(size_t i=0; i<segments2d_[0].size(); i++) {
          temp.push_back( BoostPoint(segments2d_[0][i].x()-cnt.x(), segments2d_[0][i].y()-cnt.y()));
        }

        poly.set(temp.begin(), temp.end());

        r.push_back(poly);
      }
      return r;
    }
#endif

    Eigen::Vector3f project2plane(const float x, const float y, const float mz, const float b) {
      Eigen::Vector2f mv;
      Eigen::Vector3f mv3;
      mv(0)=x;
      mv(1)=y;
      mv=mv*mz-getCenter();
      //mv(0)/=proj2plane_(0,0);
      //mv(1)/=proj2plane_(1,1);
#if DEBUG_LEVEL >70
      if(!pcl_isfinite(mv(0))) {
        ROS_ERROR("mistXYZ");
        std::cerr<<x<<","<<y<<","<<mz<<"\nmv:\n";
        std::cerr<<mv<<"\np\n";
        std::cerr<<"mz: "<<mz<<"\nmv:\n";
        //getchar();
      }
#endif
      mv3(2)=b;
      mv3(0)=mv(0);
      mv3(1)=mv(1);
      return mv3;
    }

    Eigen::Vector3f project2plane(const float x, const float y, const SubStructure::Model &model, const float b) {
      Eigen::Vector2f mv;
      Eigen::Vector3f mv3;
      mv(0)=x;
      mv(1)=y;
      mv3(2)=b;
      float mz;
      mz=model.intersect(mv(0), mv(1));

      //      Eigen::Vector3f v;
      //      v.head<2>() = mz * mv;
      //      v(2) = mz;

      mv=mv*mz-param_.col(0).head<2>();
      //      mv(0) = (v-param_.col(0)).dot(proj2plane_.col(0))/proj2plane_.col(0).squaredNorm();
      //      mv(1) = (v-param_.col(0)).dot(proj2plane_.col(1))/proj2plane_.col(1).squaredNorm();

      //      ROS_INFO("e %f %f", (v-project2world(mv)).norm(), model.model(mz*x,mz*y)-mz);

#if DEBUG_LEVEL >70
      if(!pcl_isfinite(mv(0))) {
        ROS_ERROR("mist");
        std::cerr<<x<<","<<y<<","<<mz<<"\nmv:\n";
        std::cerr<<mv<<"\np\n";
        std::cerr<<model.p<<"\n";
        std::cerr<<"mz: "<<mz<<"\n";
        //getchar();
      }
#endif
      mv3(0)=mv(0);
      mv3(1)=mv(1);
      return mv3;
    }

    Eigen::Vector2f getCenter() const {
      Eigen::Vector2f r;
      r(0)=param_(0,0);
      r(1)=param_(1,0);
      return r;
    }

    Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const {
      Eigen::Vector3f pt2;
      pt2(0)=pt(0)*pt(0);
      pt2(1)=pt(1)*pt(1);
      pt2(2)=pt(0)*pt(1);

      return param_.col(0) + proj2plane_*pt + proj2plane_.col(0).cross(proj2plane_.col(1)) * (pt2.dot(param_.col(2)) + pt.dot(param_.col(1).head<2>()));
    }

    float getFeature(Eigen::Vector2f &ra,Eigen::Vector2f &rb,Eigen::Vector3f &rc) const {

      rc=feature_;

      float a=param_(0,2), b=param_(1,2), c=param_(2,2);

      float x1 =  sqrtf((a-b)*sqrtf(c*c+b*b-2*a*b+a*a)+c*c+b*b-2*a*b+a*a)/(sqrtf(2)*sqrtf(c*c+b*b-2*a*b+a*a));
      float x2 = -sqrtf((b-a)*sqrtf(c*c+b*b-2*a*b+a*a)+c*c+b*b-2*a*b+a*a)/(sqrtf(2)*sqrtf(c*c+b*b-2*a*b+a*a));
      float y1=sqrtf(1-x1*x1);
      float y2=sqrtf(1-x2*x2);

      ra(0)=x1;
      ra(1)=y1;

      rb(0)=x2;
      rb(1)=y2;

      Eigen::Vector3f t=getInvariantCurvature();

      return std::max( std::abs(t(0)), std::max(std::abs(t(1)),std::abs(t(2))) );
    }

    Eigen::Vector3f getInvariantCurvature() const {
      Eigen::Vector3f r;
      float a=param_(0,2), b=param_(1,2), c=param_(2,2);

      float x1 =  sqrtf((a-b)*sqrtf(c*c+b*b-2*a*b+a*a)+c*c+b*b-2*a*b+a*a)/(sqrtf(2)*sqrtf(c*c+b*b-2*a*b+a*a));
      float x2 = -sqrtf((b-a)*sqrtf(c*c+b*b-2*a*b+a*a)+c*c+b*b-2*a*b+a*a)/(sqrtf(2)*sqrtf(c*c+b*b-2*a*b+a*a));
      float y1=sqrtf(1-x1*x1);
      float y2=sqrtf(1-x2*x2);

      r(0)=atanf( (a*x1*x1+b*y1*y1+c*x1*y1) /30.f);
      r(1)=atanf( (a*x2*x2+b*y2*y2+c*x2*y2) /30.f);
      r(2)=0.f;

      return r;
    }

    void update() {
      Eigen::Vector3f x,y;
      x(0)=1.f;
      y(1)=1.f;
      x(1)=x(2)=y(0)=y(2)=0.f;

      proj2plane_.col(0)=x;
      proj2plane_.col(1)=y;

      //proj2plane_.col(0)=param_.col(1).cross(y);
      //proj2plane_.col(1)=param_.col(1).cross(x);

      //param_.col(2)(0)*=proj2plane_(0,0)*proj2plane_(0,0);
      //param_.col(2)(1)*=proj2plane_(1,1)*proj2plane_(1,1);
      //param_.col(2)(2)*=proj2plane_(0,0)*proj2plane_(1,1);
    }

    void operator=(const SubStructure::Model &model) {
      feature_.fill(0);
      param_xy_ = model.p;
      model_ = model;

      //      if(std::abs(model.p(5))<0.0001f) {
      //        param_.col(0)(0)=-model.p(1)/(2*model.p(2));
      //        param_.col(0)(1)=-model.p(3)/(2*model.p(4));
      //      }
      //      else {
      //        param_.col(0)(1)=
      //            ((2*model.p(2)*model.p(3))/model.p(5)-model.p(1)) /
      //            ( model.p(5)-(4*model.p(2)*model.p(4)/model.p(5)));
      //
      //        param_.col(0)(0)=-(model.p(1)+model.p(5)*param_.col(0)(1))/(2*model.p(2));
      //      }
      //
      //      if(!pcl_isfinite(param_.col(0)(0)))
      //        param_.col(0)(0)=0.f;
      //      if(!pcl_isfinite(param_.col(0)(1)))
      //        param_.col(0)(1)=0.f;
      //
      //      param_.col(0)(2)=model.model(param_.col(0)(0), param_.col(0)(1));

      param_.col(0) = Eigen::Vector3f::Zero();
      param_.col(0)(2) = model.p(0);

      param_.col(1)(0)=model.p(1);//-(model.p(1)+2*model.p(2)*param_.col(0)(0)+model.p(5)*param_.col(0)(1));
      param_.col(1)(1)=model.p(3);//-(model.p(3)+2*model.p(4)*param_.col(0)(1)+model.p(5)*param_.col(0)(0));
      param_.col(1)(2)=0;
      //      param_.col(1).normalize();

      param_.col(2)(0)=model.p(2);
      param_.col(2)(1)=model.p(4);
      param_.col(2)(2)=model.p(5);

      weight_ = model.param.model_(0,0);

      update();

      middle_(0) = model_.param.model_(0,1)/model_.param.model_(0,0);
      middle_(1) = model_.param.model_(0,3)/model_.param.model_(0,0);
      middle_(2) = model_.param.z_(0)/model_.param.model_(0,0);

      middle_ = project2world(nextPoint(middle_));
    }

    void print() {
      std::cout<<"parameter:\n"<<param_<<"\n";
      for(size_t j=0; j<segments_.size(); j++){
        std::cout<<"---------------\n";
        for(size_t i=0; i<segments_[j].size(); i++)
          std::cout<<segments_[j][i]<<"\n";
      }
    }

    void toRosMsg(cob_3d_mapping_msgs::CurvedPolygon *msg, const ros::Time &time) const
    {
      static int nextID = 0;
      msg->stamp = time;
      msg->ID = nextID++;
      msg->weight = model_.param.model_(0,0);

      for(int i=0; i<6; i++)
        msg->parameter[i] = model_.p(i);

      msg->polyline.clear();
      if(segments_.size()>0) {
        for(size_t i=0; i<segments_[0].size(); i++) {
          cob_3d_mapping_msgs::polyline_point pt;
          pt.x=segments_[0][i](0);
          pt.y=segments_[0][i](1);
          pt.edge_prob=segments_[0][i](2);
          msg->polyline.push_back(pt);
        }
      }

      cob_3d_mapping_msgs::feature ft;

      //nearest point
      ft.ID = 1;
      Eigen::Vector3f nxtPt = project2world(nextPoint(Eigen::Vector3f::Zero()));
      ft.x=nxtPt(0);
      ft.y=nxtPt(1);
      ft.z=nxtPt(2);
      msg->features.push_back(ft);

      //feature from form
      ft.ID = 2;
      ft.x=feature_(0);
      ft.y=feature_(1);
      ft.z=feature_(2);
      msg->features.push_back(ft);

      //mid of outer hull
      ft.ID = 3;
      ft.x=middle_(0);
      ft.y=middle_(1);
      ft.z=middle_(2);
      msg->features.push_back(ft);

      Eigen::Vector2f ft1,ft2;
      Eigen::Vector3f ft3,n=model_.getNormal(middle_(0),middle_(1)),ft1p,ft2p;
      getFeature(ft1,ft2,ft3);

      ft1p(0)=ft1(0);
      ft1p(1)=ft1(1);
      ft2p(0)=ft2(0);
      ft2p(1)=ft2(1);
      ft1p(2)=ft2p(2)=0.f;

      //curvature feature 1
      ft.ID = 4;
      ft3=ft1p.cross(n);
      ft.x = ft3(0);
      ft.y = ft3(1);
      ft.z = ft3(2);
      //if(pcl_isfinite(ft3.sum()))
      msg->features.push_back(ft);

      //curvature feature 2
      ft.ID = 5;
      ft3=ft2p.cross(n);
      ft.x = ft3(0);
      ft.y = ft3(1);
      ft.z = ft3(2);
      //if(pcl_isfinite(ft3.sum()))
      msg->features.push_back(ft);

    }


    /// find nearest point to manifold (Newton)
    Eigen::Vector2f _nextPoint(const Eigen::Vector3f &v, Eigen::Vector3f p, const int depth=0) const
    {
      if(depth>10)
      {
        return p.head<2>();
      }

      //      std::cout<<"p\n"<<p<<"\n";
      p(2) = param_.col(2)(0)*p(0)*p(0)+param_.col(2)(1)*p(1)*p(1)+param_.col(2)(2)*p(0)*p(1)
                  + param_.col(1)(0)*p(0)+param_.col(1)(1)*p(1);
      //      std::cout<<"p\n"<<p<<"\n";

      Eigen::Vector3f n;
      n(0) = -(param_.col(2)(0)*2*p(0)+param_.col(2)(2)*p(1)+param_.col(1)(0));
      n(1) = -(param_.col(2)(1)*2*p(1)+param_.col(2)(2)*p(0)+param_.col(1)(1));
      n(2) = 1;

      Eigen::Vector3f d = ((p-v).dot(n)/n.squaredNorm()*n+(v-p));

      //      std::cout<<"d\n"<<d<<"\n";
      //      std::cout<<"n\n"<<n<<"\n";
      //      std::cout<<"pv\n"<<(p-v)/(p-v)(2)<<"\n";
      //      std::cout<<"pv\n"<<(p-v)<<"\n";
      //      std::cout<<"dd\n"<<(p-v).dot(n)/n.squaredNorm()*n<<"\n";
      //ROS_ASSERT(std::abs(d(2))<=std::abs(z));

      if(!pcl_isfinite(d.sum()) || d.head<2>().squaredNorm()<0.001f*0.001f)
      {
        //        std::cout<<"---------------\n";
        return (p+d).head<2>();
      }
      return _nextPoint(v,p+d,depth+1);
    }

    /// find nearest point to manifold (Newton)
    Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const
    {
      Eigen::Vector3f p;

      p(0) = (v-param_.col(0)).dot(proj2plane_.col(0));
      p(1) = (v-param_.col(0)).dot(proj2plane_.col(1));
      p(2) = (v-param_.col(0)).dot(proj2plane_.col(0).cross(proj2plane_.col(1)));

      Eigen::Vector2f r = _nextPoint(p, p);

      float e1 = (v-project2world(p.head<2>())).norm();
      float e2 = (v-project2world(r)).norm();

      if(e1<e2)
      {
        return p.head<2>();
      }

      return r;
    }
  };
}

#endif /* POLYGON_H_ */
