/*
 * polygon.h
 *
 *  Created on: 14.06.2012
 *      Author: josh
 */

#ifndef POLYGON_H_
#define POLYGON_H_

#include "sub_structures.h"

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
      mv=mv*mz-getCenter();
      mv(0)/=proj2plane_(0,0);
      mv(1)/=proj2plane_(1,1);
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
      mv(0)/=proj2plane_(0,0);
      mv(1)/=proj2plane_(1,1);
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
      mv=mv*mz-getCenter();
      mv(0)/=proj2plane_(0,0);
      mv(1)/=proj2plane_(1,1);
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

    Eigen::Vector3f project2world(const Eigen::Vector2f &pt) {
      Eigen::Vector3f pt2;
      pt2(0)=pt(0)*pt(0);
      pt2(1)=pt(1)*pt(1);
      pt2(2)=pt(0)*pt(1);

      return param_.col(0) + proj2plane_*pt + param_.col(1)*(pt2).dot(param_.col(2));
    }

    float getFeature(Eigen::Vector2f &ra,Eigen::Vector2f &rb,Eigen::Vector3f &rc) const {

      rc=feature_;

      /*ra=feature_;
      rb.fill(0);

      return 1;*/

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
#if 0
      //TESTING
      r=param_.col(2);
      r(0)=atanf(r(0)/30);
      r(1)=atanf(r(1)/30);
      r(2)=atanf(r(2)/30);
      return r;

      float x,y;
      /*float a2=param_(0,2)*param_(0,2);
      float b2=param_(1,2)*param_(1,2);
      if(std::abs(model.p(5))<0.0001f) {
        x=b2/(a2-b2);
        y=sqrtf(1-x);
      }
      else*/ {
        float a=2*param_(0,2)+param_(2,2);
        float c=param_(1,1)+param_(0,1);
        float b=2*param_(1,2)+param_(2,2);

        float ma=a*a+b*b;
        float mb=a*c;
        float mc=c*c-b*b;

        x=(mb+sqrtf(mb*mb-4*ma*mc))/(-2*ma);
        y=sqrtf(1-x*x);
      }
      /*x=sqrtf(x);
      r(0)=atanf(param_(0,2)*x+param_(1,2)*y);
      r(1)=atanf(param_(0,2)*y-param_(1,2)*x);*/
      r(0)=atanf( (2*param_(0,2)*x+2*param_(1,2)*y+param_(2,2)*(x+y)) /30.f);
      r(1)=atanf( (2*param_(0,2)*y-2*param_(1,2)*x+param_(2,2)*(y-x)) /30.f);
      float x2=M_SQRT1_2*x-M_SQRT1_2*y;
      float y2=M_SQRT1_2*x+M_SQRT1_2*y;
      r(2)=atanf( (2*param_(0,2)*x2+2*param_(1,2)*y2+param_(2,2)*(x2+y2)) /30.f);

      std::cout<<"CURVXYZ1 "<<param_(0,2)<<" "<<param_(1,2)<<" "<<param_(2,2)<<"\n";
      std::cout<<"CURVXYZ2 "<<r(0)<<" "<<r(1)<<" "<<r(2)<<"\n";

#else
      float a=param_(0,2), b=param_(1,2), c=param_(2,2);

      float x1 =  sqrtf((a-b)*sqrtf(c*c+b*b-2*a*b+a*a)+c*c+b*b-2*a*b+a*a)/(sqrtf(2)*sqrtf(c*c+b*b-2*a*b+a*a));
      float x2 = -sqrtf((b-a)*sqrtf(c*c+b*b-2*a*b+a*a)+c*c+b*b-2*a*b+a*a)/(sqrtf(2)*sqrtf(c*c+b*b-2*a*b+a*a));
      float y1=sqrtf(1-x1*x1);
      float y2=sqrtf(1-x2*x2);

      r(0)=atanf( (a*x1*x1+b*y1*y1+c*x1*y1) /30.f);
      r(1)=atanf( (a*x2*x2+b*y2*y2+c*x2*y2) /30.f);
      r(2)=0.f;

      std::cout<<"CURVXYZ1 "<<param_(0,2)<<" "<<param_(1,2)<<" "<<param_(2,2)<<"\n";
      std::cout<<"CURVXYZ2 "<<r(0)<<" "<<r(1)<<" "<<r(2)<<"\n";
#endif
      return r;
    }

    void update() {
      Eigen::Vector3f x,y;
      x(0)=1.f;
      y(1)=1.f;
      x(1)=x(2)=y(0)=y(2)=0.f;

      proj2plane_.col(0)=param_.col(1).cross(y);
      proj2plane_.col(1)=param_.col(1).cross(x);

      param_.col(2)(0)*=proj2plane_(0,0)*proj2plane_(0,0);
      param_.col(2)(1)*=proj2plane_(1,1)*proj2plane_(1,1);
      param_.col(2)(2)*=proj2plane_(0,0)*proj2plane_(1,1);
    }

    void operator=(const SubStructure::Model &model) {
      param_xy_ = model.p;
      model_ = model;

      if(std::abs(model.p(5))<0.0001f) {
        param_.col(0)(0)=-model.p(1)/(2*model.p(2));
        param_.col(0)(1)=-model.p(3)/(2*model.p(4));
      }
      else {
        param_.col(0)(1)=
            ((2*model.p(2)*model.p(3))/model.p(5)-model.p(1)) /
            ( model.p(5)-(4*model.p(2)*model.p(4)/model.p(5)));

        param_.col(0)(0)=-(model.p(1)+model.p(5)*param_.col(0)(1))/(2*model.p(2));
      }

      if(!pcl_isfinite(param_.col(0)(0)))
        param_.col(0)(0)=0.f;
      if(!pcl_isfinite(param_.col(0)(1)))
        param_.col(0)(1)=0.f;

      param_.col(0)(2)=model.model(param_.col(0)(0), param_.col(0)(1));

      param_.col(1)(0)=-(model.p(1)+2*model.p(2)*param_.col(0)(0)+model.p(5)*param_.col(0)(1));
      param_.col(1)(1)=-(model.p(3)+2*model.p(4)*param_.col(0)(1)+model.p(5)*param_.col(0)(0));
      param_.col(1)(2)=1;
      param_.col(1).normalize();

      param_.col(2)(0)=model.p(2);
      param_.col(2)(1)=model.p(4);
      param_.col(2)(2)=model.p(5);

      weight_ = model.param.model_(0,0);

      update();
    }

    void print() {
      std::cout<<"parameter:\n"<<param_<<"\n";
      for(size_t j=0; j<segments_.size(); j++){
        std::cout<<"---------------\n";
        for(size_t i=0; i<segments_[j].size(); i++)
          std::cout<<segments_[j][i]<<"\n";
      }
    }

#if 0
    void toRosMsg(cob_3d_mapping_msgs::CurvedPolygonPtr msg, const ros::Time &time, const std::vector<S_CORS> &cors) const
    {
      msg->stamp = time;
      msg->ID = form_->obj_->getId();

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
      ft.x=param_.col(0)(0);
      ft.y=param_.col(0)(1);
      ft.z=param_.col(0)(2);
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
      Eigen::Vector3f ft3,n=model_.getNormal(middle_(0),middle_(1));
      getFeature(ft1,ft2,ft3);

      //curvature feature 1
      ft.ID = 4;
      ft.x=ft1(0);
      ft.y=ft1(1);
      ft.z= -(ft1.dot(n.head<2>()))/n(2);
      msg->features.push_back(ft);

      //curvature feature 2
      ft.ID = 5;
      ft.x=ft2(0);
      ft.y=ft2(1);
      ft.z= -(ft2.dot(n.head<2>()))/n(2);
      msg->features.push_back(ft);


      msg->score;
      Classification::ObjectList::Ptr ol=Classification::ObjectSearchTree::get().findList(form_);

      for(size_t i=0; i<ol->get().size(); i++) {
        cob_3d_mapping_msgs::simalarity_score sc;

        sc.prob = ol->get()[i].dist_;
        sc.ID = ol->get()[i].n_->obj_->getId();

        msg->score.push_back(sc);
      }

      //add cors
      for(size_t i=0; i<cors.size(); i++) {
        cob_3d_mapping_msgs::simalarity_score sc;

        sc.prob = cors[i].prob_;
        sc.ID = cors[i].ID_;

        msg->score.push_back(sc);
      }

      TiXmlElement *root=Classification::Serializable::createRoot();
      form_->writeXML(root);
      msg->energy = Classification::Serializable::getString(root);

      for(int i=0; i<connectivity_.connections_.size(); i++)
        std::cout<<"CON bt "<<i<<" with "<<connectivity_.connections_[i]<<"\n";
    }
#endif

#if 0
    static void frontal_outline(const std::vector<S_POLYGON> &o, std::vector<Eigen::Vector2f> &pts, Eigen::Vector2f dir) {
      pts.clear();

      if(segments_.size()<1 || o.segments_.size()<1 || segments_[0].size()<2 || o.segments_[0].size()<2) return; //only outer

      /*dir(0)=(o.param_xy_(1)-param_xy_(1))/2;
      float f=(param_xy_(0)-o.param_xy_(0))/(o.param_xy_(4)-param_xy_(4));
      if(f<0.f) return;
      dir(1)=sqrtf(f);*/
      dir.normalize();
      /*Eigen::Vector2f off;
      off(0)=0.f;
      off(1)=(o.param_xy_(3)-param_xy_(3))/2;*/

      Eigen::Vector2f v;
      v(0)=dir(1);
      v(1)=-dir(0);

      //step 1: project to v
      float min_A=std::numeric_limits<float>::min(), max_A=std::numeric_limits<float>::max();

      for(size_t j=0; j<o.size(); j++) {
        for(size_t i=0; i<o[j].segments_[0].size(); i++) {
          f=v.dot(o[ji].segments_[0][i].head<2>());
          min_A=std::min(f,min_B);
          max_A=std::max(f,max_B);
        }
      }

      for(size_t i=0; i<o.size(); i++) {
        //step 2: curvature in dir + number of pts
        float c_B=dir(0)*o[i].param_xy_(2)+dir(1)*o[i].param_xy_(4);
        int n_B=2+(int)(std::abs(c_B)*(max_B-min_B)*20);
        std::cout<<"trying "<<n_B<<" points\n";
      }

      /*//check if it's near and in wich way (left, right connected)
      const float thr=0.03;
      if(std::min(std::abs(max_A-max_B), std::min(std::abs(max_A-min_B),
                                                  std::min(std::abs(min_A-max_B), std::abs(min_A-max_B))))
      >thr) //TODO: check threshold
        return;
      bool reverse_A=false,reverse_B=false;
      bool closed=false;
      if(std::min(std::abs(max_A-max_B), std::abs(max_A-min_B))<=thr) {
        reverse_B = (std::abs(max_A-max_B)<std::abs(max_A-min_B));
        closed=std::min(std::abs(min_A-max_B), std::abs(min_A-max_B))<=thr;
      }
      else {
        reverse_A = true;
        reverse_B = (std::abs(min_A-max_B)<std::abs(min_A-min_B));
      }
      //step 2: curvature in dir + number of pts
      float c_A=dir(0)*param_xy_(2)+dir(1)*param_xy_(4);
      float c_B=dir(0)*o.param_xy_(2)+dir(1)*o.param_xy_(4);
      int n_A=2+(int)(std::abs(c_A)*(max_A-min_A)*20);
      std::cout<<"trying "<<n_A<<" points\n";
      int n_B=2+(int)(std::abs(c_B)*(max_B-min_B)*20);
      std::cout<<"trying "<<n_B<<" points\n";

      //step 3: create pts
      Eigen::Vector2f p;
      for(int i=reverse_A?n_A-1:0; i!=(reverse_A?-1:n_A); reverse_A?i--:i++) {
        p(0)=i*(max_A-min_A)/n_A + min_A;
        p(1)=dir(1)*p(0)+off(1);
        p(1)=param_xy_(0)+param_xy_(1)*p(0)+param_xy_(2)*p(0)*p(0)+param_xy_(3)*p(1)+param_xy_(4)*p(1)*p(1);
        pts.push_back(p);
      }
      for(int i=reverse_B?n_B-1:0; i!=(reverse_B?-1:n_B); reverse_B?i--:i++) {
        p(0)=i*(max_B-min_B)/n_B + min_B;
        p(1)=dir(1)*p(0)+off(1);
        p(1)=o.param_xy_(0)+o.param_xy_(1)*p(0)+o.param_xy_(2)*p(0)*p(0)+o.param_xy_(3)*p(1)+o.param_xy_(4)*p(1)*p(1);
        pts.push_back(p);
      }*/
    }
#endif
  };
  }

#endif /* POLYGON_H_ */
