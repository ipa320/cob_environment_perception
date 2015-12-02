/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: cob_3d_segmenation
 * \note
 *  ROS stack name: cob_3d_environment_perception
 * \note
 *  ROS package name: cob_3d_segmenation
 *
 * \author
 *  Author: Joshua Hampp (joshua.hampp@ipa.fraunhofer.de)
 *
 * \date 2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
/*
 * polygon.h
 *
 *  Created on: 14.06.2012
 *      Author: josh
 */

#ifndef POLYGON_H_
#define POLYGON_H_

#include "sub_structures.h"
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <cob_3d_mapping_msgs/CurvedPolygon.h>
#include <libpolypartition/polypartition.h>

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

  template<int Degree>
  struct S_POLYGON {
	enum {DEGREE=Degree};
	typedef SubStructure::Model<Degree> Model;
	
    std::vector<std::vector<Eigen::Vector3f> >  segments_;
    std::vector<std::vector<Eigen::Vector2i> >  segments2d_;
#ifdef USE_BOOST_POLYGONS_
    std::vector<std::vector<BoostPoint> >       segments2d_;
#endif
    float weight_; //number of points used for this model
#ifdef CHECK_CONNECTIVITY
    S_POLYGON_CONNECTIVITY connectivity_;
#endif
    SubStructure::Model<Degree> model_;
    Eigen::Vector3f middle_;
    int mark_;

    typename SubStructure::Param<Degree>::VectorU param_xy_;
    sensor_msgs::ImagePtr img_;
    float color_[3];

#ifdef USE_CLASSIFICATION_
    Classification::Form::Ptr form_;
#endif

    Eigen::Vector3f feature_;

    S_POLYGON()
#ifdef USE_CLASSIFICATION_
    :form_(new Classification::Form)
#endif
    {
      int color = rand();
      color_[0] = ((color>>0)&0xff)/255.f;
      color_[1] = ((color>>8)&0xff)/255.f;
      color_[2] = ((color>>16)&0xff)/255.f;
    }

    bool isLinear() const
    {
      for(size_t i=3; i<SubStructure::Param<Degree>::NUM; i++)
        if(std::abs(model_.p(i))>0.01f) return false;
      return true;
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

    Eigen::Vector2f project2plane(const float x, const float y, const SubStructure::Model<Degree> &model) {
      ROS_ERROR("TODO:");
      Eigen::Vector2f mv;
      mv(0)=x;
      mv(1)=y;
      float mz;
      mz=model.intersect(mv(0), mv(1));
      //mv=mv*mz-getCenter();
      Eigen::Vector3f v;
      v.head<2>() = mz * mv;
      v(2) = mz;
      //mv(0) = (v-param_.col(0)).dot(proj2plane_.col(0))/proj2plane_.col(0).squaredNorm();
      //mv(1) = (v-param_.col(0)).dot(proj2plane_.col(1))/proj2plane_.col(1).squaredNorm();
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
      ROS_ERROR("TODO:");//TODO:
      return Eigen::Vector3f::Zero();
      /*Eigen::Vector2f mv;
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
      return mv3;*/
    }

    Eigen::Vector3f project2plane(const float x, const float y, const SubStructure::Model<Degree> &model, const float b) {
      Eigen::Vector2f mv;
      Eigen::Vector3f mv3;
      mv(0)=x;
      mv(1)=y;
      float mz = model.intersect(mv(0), mv(1));

      mv3(0) = mv(0)*mz;
      mv3(1) = mv(1)*mz;
      mv3(2) = mz;

      return mv3;
    }

    /*Eigen::Vector2f getCenter() const {
      Eigen::Vector2f r;
      r(0)=param_(0,0);
      r(1)=param_(1,0);
      return r;
    }*/

    Eigen::Vector3f project2world(const Eigen::Vector2f &pt) const {
      Eigen::Vector3f v;
      v.head<2>() = pt;
      v(2) = model_.model(pt(0),pt(1));
      return v;
    }
  
  /**
   * get orientation at center like:
   *  - x --> normal
   *  - y --> greatest eigenvalue
   */
   inline Eigen::Quaternionf get_orientation() const {
	   Eigen::Vector2f mid, dir, mean; mid(0) = model_.x(); mid(1) = model_.y();
	   mean(0) = model_.param.model_(0,1);
	   mean(1) = model_.param.model_(0,2);
	   Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> evd (model_.param.model_.block(1,1, 2,2)-mean*mean.transpose()/model_.param.model_(0,0));
	   Eigen::Vector3f x = model_.getNormal(model_.x(), model_.y());
	   dir(0) = evd.eigenvectors().col(1)(1);
	   dir(1) = evd.eigenvectors().col(1)(0);

	   Eigen::Vector3f y = project2world(0.1f*dir+mid)-project2world(mid);
	   y.normalize();
	   
	   Eigen::Matrix3f M;
	   M.col(0) = -x.cross(y).normalized();
	   M.col(1) = y;
	   M.col(2) = x;	//normal
	   
	   return Eigen::Quaternionf(M);
   }

    Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const {
      return model_.getNormal(v(0),v(1));
    }

    /*float getFeature(Eigen::Vector2f &ra,Eigen::Vector2f &rb,Eigen::Vector3f &rc) const {

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
    }*/

    void update() {
      Eigen::Vector3f x,y;
      x(0)=1.f;
      y(1)=1.f;
      x(1)=x(2)=y(0)=y(2)=0.f;
    }

    void operator=(const SubStructure::Model<Degree> &model) {
      feature_.fill(0);
      param_xy_ = model.p;
      model_ = model;
      weight_ = model.param.model_(0,0);

      update();

      middle_(0) = model_.x();
      middle_(1) = model_.y();
      middle_(2) = model_.z();

      middle_ = project2world(nextPoint(middle_));
    }

    void print() {
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

      for(int i=0; i<model_.p.rows(); i++)      //TODO: check
        msg->parameter[i] = model_.p(i);

      msg->polyline.clear();
      if(segments_.size()>0) {
        for(size_t i=0; i<segments_[0].size(); i++) {
          cob_3d_mapping_msgs::PolylinePoint pt;
          pt.x=segments_[0][i](0);
          pt.y=segments_[0][i](1);
          pt.edge_prob=segments_[0][i](2);
          msg->polyline.push_back(pt);
        }
      }

      cob_3d_mapping_msgs::Feature ft;

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
      //getFeature(ft1,ft2,ft3);        //TODO:

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
//    Eigen::Vector2f _nextPoint(const Eigen::Vector3f &v, Eigen::Vector3f p, const int depth=0) const
//    {
//      if(depth>10)
//      {
//        return p.head<2>();
//      }
//
//      //      std::cout<<"p\n"<<p<<"\n";
//      p(2) = param_.col(2)(0)*p(0)*p(0)+param_.col(2)(1)*p(1)*p(1)+param_.col(2)(2)*p(0)*p(1)
//                  + param_.col(1)(0)*p(0)+param_.col(1)(1)*p(1);
//      //      std::cout<<"p\n"<<p<<"\n";
//
//      Eigen::Vector3f n;
//      n(0) = -(param_.col(2)(0)*2*p(0)+param_.col(2)(2)*p(1)+param_.col(1)(0));
//      n(1) = -(param_.col(2)(1)*2*p(1)+param_.col(2)(2)*p(0)+param_.col(1)(1));
//      n(2) = 1;
//
//      Eigen::Vector3f d = ((p-v).dot(n)/n.squaredNorm()*n+(v-p));
//
//      //      std::cout<<"d\n"<<d<<"\n";
//      //      std::cout<<"n\n"<<n<<"\n";
//      //      std::cout<<"pv\n"<<(p-v)/(p-v)(2)<<"\n";
//      //      std::cout<<"pv\n"<<(p-v)<<"\n";
//      //      std::cout<<"dd\n"<<(p-v).dot(n)/n.squaredNorm()*n<<"\n";
//      //ROS_ASSERT(std::abs(d(2))<=std::abs(z));
//
//      if(!pcl_isfinite(d.sum()) || d.head<2>().squaredNorm()<0.001f*0.001f)
//      {
//        //        std::cout<<"---------------\n";
//        return (p+d).head<2>();
//      }
//      return _nextPoint(v,p+d,depth+1);
//    }

    struct MyFunctor
    {
      const SubStructure::Model<Degree> &model;
      const float x0,y0,z0;

      int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
      {
        // distance
        fvec(0) = std::pow(model.model(x(0),x(1))-z0,2)+std::pow(x(1)-y0,2)+std::pow(x(0)-x0,2);
        fvec(1) = 0;

        //        std::cout<<"fvec\n"<<fvec<<std::endl;
        return 0;
      }

      int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
      {
        //Jacobian
        const float delta = model.model(x(0),x(1))-z0;
        const Eigen::Vector3f n = model.getNormal_(x(0),x(1));
        fjac(0,0) = n(0)*delta+2*(x(0)-x0);
        fjac(0,1) = n(1)*delta+2*(x(1)-y0);
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
      MyFunctor functor={model_,
                         p(0),p(1),p(2)};
      Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);
      lm.parameters.maxfev = 50;
      lm.minimize(r);

      return r;
    }

    /// find nearest point to manifold (Newton)
    Eigen::Vector2f nextPoint(const Eigen::Vector3f &v) const
    {
      /*Eigen::Vector3f p, pos0 = project2world(v.head<2>()), n0 = normalAt(v.head<2>()), n0x, n0y;

      n0x = n0y = model_.getNormal_(v(0),v(1));
      n0x(1) = 0;
      n0y(0) = 0;
      n0x.normalize();
      n0y.normalize();

      p(0) = (v-pos0).dot(n0x);
      p(1) = (v-pos0).dot(n0y);
      p(2) = (v-pos0).dot(n0);*/

      Eigen::Vector2f r = _nextPoint(v, v);

      float e1 = (v-project2world(v.head<2>())).norm();
      float e2 = (v-project2world(r)).norm();

      if(e1<e2)
      {
        return v.head<2>();
      }

      return r;
    }

    bool getPose(Eigen::Matrix3f &P, Eigen::Vector3f &origin, float &h, float &w) const {
      if(segments_.size()<1)
        return false;

      Eigen::Matrix2f A, C;
      Eigen::Vector2f B, v1, v2;

      A(0,0) = model_.param.model_(1,1);
      A(1,1) = model_.param.model_(3,3);
      A(0,1) = A(1,0) = model_.param.model_(1,3); //TODO: fix

      B(0) = model_.param.model_(0,1);
      B(1) = model_.param.model_(0,3);

      C = A-C*C.transpose();

      Eigen::EigenSolver<Eigen::Matrix2f> es(C);

//      std::cout<<"EV1\n"<<es.eigenvectors().col(0)<<"\n";
//      std::cout<<"EV2\n"<<es.eigenvectors().col(1)<<"\n";

      v1 = es.eigenvectors().col(0).real();
      v2 = es.eigenvectors().col(1).real();

      v1.normalize();
      v2.normalize();

      P.col(0) = project2world(v1)-project2world(Eigen::Vector2f::Zero());
      P.col(1) = project2world(v2)-project2world(Eigen::Vector2f::Zero());
      P.col(0).normalize();
      P.col(1).normalize();
      P.col(2) = P.col(0).cross(P.col(1));

      float h_ma = -std::numeric_limits<float>::max(), h_mi = std::numeric_limits<float>::max();
      float w_ma = -std::numeric_limits<float>::max(), w_mi = std::numeric_limits<float>::max();

      for(size_t i=0; i<segments_[0].size(); i++)
      {
        float f=segments_[0][i].head<2>().dot(v1);
        h_ma = std::max(h_ma, f);
        h_mi = std::min(h_mi, f);

        f=segments_[0][i].head<2>().dot(v2);
        w_ma = std::max(w_ma, f);
        w_mi = std::min(w_mi, f);
      }

      origin = middle_;
      w = (project2world(v1*h_ma)-project2world(v1*h_mi)).norm();
      h = (project2world(v2*w_ma)-project2world(v2*w_mi)).norm();

//      ROS_INFO("%f %f   %f %f", w, h_ma-h_mi, h, w_ma-w_mi);

      return true;
    }

    float area() const {

      TPPLPartition pp;
      std::list<TPPLPoly> polys,result;

      //std::cout << "id: " << new_message->id << std::endl;
      //std::cout << new_message->centroid << std::endl;
      //fill polys
      for(size_t i=0; i<segments_.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        TPPLPoly poly;

        poly.Init(segments_[i].size());
        poly.SetHole(i!=0);

        for(size_t j=0; j<segments_[i].size(); j++) {
          poly[j].x = segments_[i][j](0);
          poly[j].y = segments_[i][j](1);
        }
        if(i!=0)
          poly.SetOrientation(TPPL_CW);
        else
          poly.SetOrientation(TPPL_CCW);

        polys.push_back(poly);
      }

      pp.Triangulate_EC(&polys,&result);

      TPPLPoint p1,p2,p3;
      Eigen::Vector3f v1,v2,v3;

      float a=0.f;
      for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
        if(it->GetNumPoints()!=3) continue;

        p1 = it->GetPoint(0);
        p2 = it->GetPoint(1);
        p3 = it->GetPoint(2);

        v1(0) = p1.x;
        v1(1) = p1.y;
        v2(0) = p2.x;
        v2(1) = p2.y;
        v3(0) = p3.x;
        v3(1) = p3.y;

        v1 = project2world(v1.head<2>());
        v2 = project2world(v2.head<2>());
        v3 = project2world(v3.head<2>());

        a += (v2-v1).cross(v3-v1).norm();
      }

      return a;
    }

  };

#ifdef QPPF_SPECIALIZATION_2
  template< >
  struct S_POLYGON<2> {
    Eigen::Matrix3f param_;
    std::vector<std::vector<Eigen::Vector3f> >  segments_;
    std::vector<std::vector<Eigen::Vector2i> >  segments2d_;
#ifdef USE_BOOST_POLYGONS_
    std::vector<std::vector<BoostPoint> >       segments2d_;
#endif
    float weight_; //number of points used for this model
#ifdef CHECK_CONNECTIVITY
    S_POLYGON_CONNECTIVITY connectivity_;
#endif
    SubStructure::Model<2> model_;
    Eigen::Vector3f middle_;
    int mark_;

    Eigen::Matrix<float,3,2> proj2plane_;
    typename SubStructure::Param<2>::VectorU param_xy_;
    sensor_msgs::ImagePtr img_;
    float color_[3];

#ifdef USE_CLASSIFICATION_
    Classification::Form::Ptr form_;
#endif

    Eigen::Vector3f feature_;

    S_POLYGON()
#ifdef USE_CLASSIFICATION_
    :form_(new Classification::Form)
#endif
    {
      int color = rand();
      color_[0] = ((color>>0)&0xff)/255.f;
      color_[1] = ((color>>8)&0xff)/255.f;
      color_[2] = ((color>>16)&0xff)/255.f;
    }

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

    Eigen::Vector2f project2plane(const float x, const float y, const SubStructure::Model<2> &model) {
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

    Eigen::Vector3f project2plane(const float x, const float y, const SubStructure::Model<2> &model, const float b) {
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

    Eigen::Vector3f normalAt(const Eigen::Vector2f &v) const {
      Eigen::Vector3f r;

      r(0) = -(param_.col(1)(0) + 2*v(0)*param_.col(2)(0)+v(1)*param_.col(2)(2));
      r(1) = -(param_.col(1)(1) + 2*v(1)*param_.col(2)(1)+v(0)*param_.col(2)(2));
      r(2) = 1;

      r.normalize();

      Eigen::Matrix3f M;
      M.col(0) = proj2plane_.col(0);
      M.col(1) = proj2plane_.col(1);
      M.col(2) = proj2plane_.col(0).cross(proj2plane_.col(1));

      return M*r;
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

    void operator=(const SubStructure::Model<2> &model) {
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

      middle_(0) = model_.x();
      middle_(1) = model_.y();
      middle_(2) = model_.z();

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

      for(int i=0; i<model_.p.rows(); i++)      //TODO: check
        msg->parameter[i] = model_.p(i);

      msg->polyline.clear();
      if(segments_.size()>0) {
        for(size_t i=0; i<segments_[0].size(); i++) {
          cob_3d_mapping_msgs::PolylinePoint pt;
          pt.x=segments_[0][i](0);
          pt.y=segments_[0][i](1);
          pt.edge_prob=segments_[0][i](2);
          msg->polyline.push_back(pt);
        }
      }

      cob_3d_mapping_msgs::Feature ft;

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
//    Eigen::Vector2f _nextPoint(const Eigen::Vector3f &v, Eigen::Vector3f p, const int depth=0) const
//    {
//      if(depth>10)
//      {
//        return p.head<2>();
//      }
//
//      //      std::cout<<"p\n"<<p<<"\n";
//      p(2) = param_.col(2)(0)*p(0)*p(0)+param_.col(2)(1)*p(1)*p(1)+param_.col(2)(2)*p(0)*p(1)
//                  + param_.col(1)(0)*p(0)+param_.col(1)(1)*p(1);
//      //      std::cout<<"p\n"<<p<<"\n";
//
//      Eigen::Vector3f n;
//      n(0) = -(param_.col(2)(0)*2*p(0)+param_.col(2)(2)*p(1)+param_.col(1)(0));
//      n(1) = -(param_.col(2)(1)*2*p(1)+param_.col(2)(2)*p(0)+param_.col(1)(1));
//      n(2) = 1;
//
//      Eigen::Vector3f d = ((p-v).dot(n)/n.squaredNorm()*n+(v-p));
//
//      //      std::cout<<"d\n"<<d<<"\n";
//      //      std::cout<<"n\n"<<n<<"\n";
//      //      std::cout<<"pv\n"<<(p-v)/(p-v)(2)<<"\n";
//      //      std::cout<<"pv\n"<<(p-v)<<"\n";
//      //      std::cout<<"dd\n"<<(p-v).dot(n)/n.squaredNorm()*n<<"\n";
//      //ROS_ASSERT(std::abs(d(2))<=std::abs(z));
//
//      if(!pcl_isfinite(d.sum()) || d.head<2>().squaredNorm()<0.001f*0.001f)
//      {
//        //        std::cout<<"---------------\n";
//        return (p+d).head<2>();
//      }
//      return _nextPoint(v,p+d,depth+1);
//    }

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

    bool getPose(Eigen::Matrix3f &P, Eigen::Vector3f &origin, float &h, float &w) const {
      if(segments_.size()<1)
        return false;

      Eigen::Matrix2f A, C;
      Eigen::Vector2f B, v1, v2;

      A(0,0) = model_.param.model_(1,1);
      A(1,1) = model_.param.model_(3,3);
      A(0,1) = A(1,0) = model_.param.model_(1,3); //TODO: fix

      B(0) = model_.param.model_(0,1);
      B(1) = model_.param.model_(0,3);

      C = A-C*C.transpose();

      Eigen::EigenSolver<Eigen::Matrix2f> es(C);

//      std::cout<<"EV1\n"<<es.eigenvectors().col(0)<<"\n";
//      std::cout<<"EV2\n"<<es.eigenvectors().col(1)<<"\n";

      v1 = es.eigenvectors().col(0).real();
      v2 = es.eigenvectors().col(1).real();

      v1.normalize();
      v2.normalize();

      P.col(0) = project2world(v1)-project2world(Eigen::Vector2f::Zero());
      P.col(1) = project2world(v2)-project2world(Eigen::Vector2f::Zero());
      P.col(0).normalize();
      P.col(1).normalize();
      P.col(2) = P.col(0).cross(P.col(1));

      float h_ma = -std::numeric_limits<float>::max(), h_mi = std::numeric_limits<float>::max();
      float w_ma = -std::numeric_limits<float>::max(), w_mi = std::numeric_limits<float>::max();

      for(size_t i=0; i<segments_[0].size(); i++)
      {
        float f=segments_[0][i].head<2>().dot(v1);
        h_ma = std::max(h_ma, f);
        h_mi = std::min(h_mi, f);

        f=segments_[0][i].head<2>().dot(v2);
        w_ma = std::max(w_ma, f);
        w_mi = std::min(w_mi, f);
      }

      origin = middle_;
      w = (project2world(v1*h_ma)-project2world(v1*h_mi)).norm();
      h = (project2world(v2*w_ma)-project2world(v2*w_mi)).norm();

//      ROS_INFO("%f %f   %f %f", w, h_ma-h_mi, h, w_ma-w_mi);

      return true;
    }

    float area() const {

      TPPLPartition pp;
      std::list<TPPLPoly> polys,result;

      //std::cout << "id: " << new_message->id << std::endl;
      //std::cout << new_message->centroid << std::endl;
      //fill polys
      for(size_t i=0; i<segments_.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        TPPLPoly poly;

        poly.Init(segments_[i].size());
        poly.SetHole(i!=0);

        for(size_t j=0; j<segments_[i].size(); j++) {
          poly[j].x = segments_[i][j](0);
          poly[j].y = segments_[i][j](1);
        }
        if(i!=0)
          poly.SetOrientation(TPPL_CW);
        else
          poly.SetOrientation(TPPL_CCW);

        polys.push_back(poly);
      }

      pp.Triangulate_EC(&polys,&result);

      TPPLPoint p1,p2,p3;
      Eigen::Vector3f v1,v2,v3;

      float a=0.f;
      for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
        if(it->GetNumPoints()!=3) continue;

        p1 = it->GetPoint(0);
        p2 = it->GetPoint(1);
        p3 = it->GetPoint(2);

        v1(0) = p1.x;
        v1(1) = p1.y;
        v2(0) = p2.x;
        v2(1) = p2.y;
        v3(0) = p3.x;
        v3(1) = p3.y;

        v1 = project2world(v1.head<2>());
        v2 = project2world(v2.head<2>());
        v3 = project2world(v3.head<2>());

        a += (v2-v1).cross(v3-v1).norm();
      }

      return a;
    }

  };
#endif
}

#endif /* POLYGON_H_ */

