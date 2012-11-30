/*
 * object.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef SCP_EXTENDED_CURVED_POLYGON_H_
#define SCP_EXTENDED_CURVED_POLYGON_H_

#define DEBUG_LEVEL 30

//#define DEBUG_SVG 1

#include <cob_3d_mapping_msgs/CurvedPolygon.h>
#include <cob_3d_mapping_msgs/Shape.h>
#include "/home/josh/workspace/dynamic_tutorials/common/include/registration/object.hpp"
//#include "sac_model_correspondence.h"
#include "form.h"
#include "polygon_merger.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "surface.h"
#include "id.h"
#include "bb.h"

#include "cob_3d_mapping_slam/lib/polypartition.h"
#include "cob_3d_mapping_slam/dof/feature.h"

namespace Slam_CurvedPolygon
{

  class ex_curved_polygon
  {
  public:

    //typedef Slam_Surface::SurfaceNurbs SURFACE;
    typedef Slam_Surface::SurfaceTriSpline SURFACE;
    typedef BoundingBox::OOBB BB;

    Slam_Surface::Surface::SWINDOW getWindow() const {
      Slam_Surface::Surface::SWINDOW w;
      w.min_x = w.min_y = std::numeric_limits<float>::max();
      w.max_x = w.max_y = -std::numeric_limits<float>::max();
      for(size_t i=0; i<data_.polyline.size(); i++)
      {
        w.min_x = std::min(w.min_x,data_.polyline[i].x);
        w.min_y = std::min(w.min_y,data_.polyline[i].y);
        w.max_x = std::max(w.max_x,data_.polyline[i].x);
        w.max_y = std::max(w.max_y,data_.polyline[i].y);
      }
      return w;
    }

  private:

    S_ID ID_;
    Eigen::Vector3f mid_point_;
    cob_3d_mapping_msgs::CurvedPolygon data_;
    Classification::Form form_;

    boost::shared_ptr<SURFACE> surface_;

    Outline outline_;
    std::vector<Eigen::Vector3f> points3d_;
    BB bb_;

    ::std_msgs::ColorRGBA color_;

    inline float modelAt(const float x, const float y)
    {
      return data_.parameter[0]+data_.parameter[1]*x+data_.parameter[2]*x*x+data_.parameter[3]*y+data_.parameter[4]*y*y+data_.parameter[5]*x*y;
    }

    void update() {
      ID_ += data_.ID;

      //std::cout<<"ID: "<<data_.ID<<"\n";
      for(size_t i=0; i<data_.score.size(); i++)
      {
        //std::cout<<data_.score[i].ID<<"\n";
        ID_ += data_.score[i].ID;
      }

      outline_.segments_.clear();
      for(size_t i=0; i<data_.polyline.size(); i++)
      {
        Eigen::Vector3f pt3;
        pt3(0) = data_.polyline[i].x;
        pt3(1) = data_.polyline[i].y;
        pt3(2) = data_.polyline[i].edge_prob;
        outline_ += pt3;
      }

      Classification::Classification::MainPoints mains;
      mains.size_ = 4;
      Classification::Classification cl;
      Classification::QuadBB qbb;
      cl.setPoints(outline_.segments_);
      Classification::Classification::MainPoints compressed;
      form_.n_ = cl.buildLocalTree(false,&qbb,&compressed,NULL,&mains);
      if(form_.n_)
        form_.n_->clean(1,form_.n_->energy_sum_*0.01f);

#ifdef DEBUG_OUTPUT_
      ROS_INFO("COMPRESSION %d %d",data_.polyline.size(),compressed.points_.size());
#endif

      data_.polyline.clear();

      for(size_t i=0; i<compressed.points_.size(); i++)
      {
        cob_3d_mapping_msgs::polyline_point pt;
        pt.x = outline_.segments_[compressed.points_[i].index_](0);
        pt.y = outline_.segments_[compressed.points_[i].index_](1);
        pt.edge_prob = compressed.points_[i].edge_;
        data_.polyline.push_back(pt);
      }

      {
        Slam_Surface::Surface::SWINDOW w=getWindow();
#ifdef INIT0
        if(data_.polyline.size())
          surface_->init(data_.parameter ,w.min_x,w.max_x, w.min_y,w.max_y, data_.weight);
        else {
          surface_->init(data_.parameter ,0,1,0,1,data_.weight);
          ROS_WARN("empty");
        }

        Slam_Surface::PolynomialSurface ps;
        ps.init(data_.parameter ,w.min_x,w.max_x, w.min_y,w.max_y, data_.weight);
        for(size_t i=0; i<data_.polyline.size(); i++)
        {
          Eigen::Vector2f p;
          p(0) = data_.polyline[i].x;
          p(1) = data_.polyline[i].y;
          p = surface_->nextPoint(ps.project2world(p));
          data_.polyline[i].x = p(0);
          data_.polyline[i].y = p(1);
        }

        PolygonData p1;
        for(size_t i=0; i<data_.polyline.size(); i++)
        {
          p1.add(data_.polyline[i].x,data_.polyline[i].y);
        }
        data_.weight *= 100*100/area(p1);
#else
        Slam_Surface::PolynomialSurface ps;
        ps.init(data_.parameter ,w.min_x,w.max_x, w.min_y,w.max_y, data_.weight);

        surface_->init(&ps, outline_.get());
#endif
      }

      update_points3d();

      //      Eigen::Vector3f n,z=Eigen::Vector3f::Zero();
      //      z(2)=1;
      //      n=z;
      //      n(0)=-data_.parameter[1];
      //      n(1)=-data_.parameter[3];
      //      n.normalize();

#ifdef DEBUG_SVG
      char buf[128];
      static int cnt=0;
      sprintf(buf,"dbg%d.svg",cnt++);
      outline_.debug_svg(buf);
#endif

      //      std::cout<<"NP\n"<<nearest_point_<<"\n";
      //      std::cout<<"NP*\n"<<data_.features[0].x<<"\n"<<data_.features[0].y<<"\n"<<data_.features[0].z<<"\n";
    }

    void update_points3d()
    {
      points3d_.clear();
      outline_.segments_.clear();

      mid_point_ = Eigen::Vector3f::Zero();
      for(size_t i=0; i<data_.polyline.size(); i++)
      {
        Eigen::Vector2f pt;
        Eigen::Vector3f pt3;
        pt3(0) = pt(0) = data_.polyline[i].x;
        pt3(1) = pt(1) = data_.polyline[i].y;
        pt3(2) = data_.polyline[i].edge_prob;
        outline_ += pt3;
        points3d_.push_back( surface_->project2world(pt) );
        //        ROS_ERROR("%f %f",pt(0),pt(1));
        //        ROS_ERROR("%f %f %f %f %f %f",data_.parameter[0],data_.parameter[1],data_.parameter[2],data_.parameter[3],data_.parameter[4],data_.parameter[5]);
        //        ROS_ERROR("%f %f",points3d_.back()(2), modelAt(pt(0),pt(1)));
        //        ROS_ASSERT( std::abs(points3d_.back()(2)-modelAt(pt(0),pt(1))) < 0.01f );
        mid_point_ += points3d_.back();
      }
      mid_point_/=data_.polyline.size();

      update_BB();
    }

    void update_BB()
    {
      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::PointXYZ pt;

      for(size_t i=0; i<outline_.size(); i++)
      {
        Eigen::Vector3f v = surface_->project2world(outline_[i].head<2>());
        pt.x = v(0);
        pt.y = v(1);
        pt.z = v(2);
        pc.push_back(pt);

        v = surface_->project2world((outline_[(i+1)%outline_.size()].head<2>()+outline_[i].head<2>())/2);
        pt.x = v(0);
        pt.y = v(1);
        pt.z = v(2);
        pc.push_back(pt);
      }

      bb_.set(pc);
    }

  public:
    ex_curved_polygon(const cob_3d_mapping_msgs::CurvedPolygon& data):
      data_(data), surface_(new SURFACE) //, form_(data.energy)
    {
      update();

      int rnd = rand();
      color_.r = ((rnd>>0)&0xff)/255.f;
      color_.g = ((rnd>>8)&0xff)/255.f;
      color_.b = ((rnd>>16)%0xff)/255.f;
      color_.a = 1.f;
    }

    ex_curved_polygon(const ex_curved_polygon &o)
    {
      *this = o;
    }

    ex_curved_polygon &operator=(const ex_curved_polygon &o)
    {
      color_ = o.color_;
      ID_ = o.ID_;
      data_ = o.data_;
      form_ = o.form_;
      mid_point_ = o.mid_point_;
      outline_ = o.outline_;
      points3d_ = o.points3d_;
      bb_ = o.bb_;
      surface_.reset(new SURFACE(*o.surface_));
    }

    virtual ~ex_curved_polygon()
    {
    }

    const BB &getBB() const
    {
      return bb_;
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const float var_R, const float var_T)
    {
      surface_->transform(rot,tr);

      //TODO:
      //nearest_point_ = rot*nearest_point_;
      //      for(size_t i=0; i<points3d_.size(); i++)
      //      {
      //        points3d_[i] = rot*points3d_[i];
      //      }

      for(size_t i=0; i<points3d_.size(); i++)
      {
        points3d_[i] = rot*points3d_[i]+tr;
      }

      mid_point_ = rot*mid_point_ + tr;

      update_BB();
      //      bb_min_ = rot*bb_min_;
      //      bb_max_ = rot*bb_max_;
      //      bb_min_+=tr;
      //      bb_max_+=tr;
    }

    inline const S_ID &getID() const {return ID_;}

    Eigen::Vector3f getNearestPoint() const {return mid_point_;}
    Eigen::Vector3f getNearestTransformedPoint(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const
    {
      return rot*mid_point_+tr;
    }

    const std::vector<cob_3d_mapping_msgs::simalarity_score> &getScore() const {return data_.score;}

    /**
     * checks intersection of two bounding boxes
     *
     * "add" adds value, so flat planes can intersect too

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
    }*/

    bool fitsCurvature(const ex_curved_polygon &o, const float thr) const
    {
      //      if( param_.col(2).squaredNorm()<0.1 && o.param_.col(2).squaredNorm()<0.1)
      //        return true;
      return surface_->fitsCurvature(*o.surface_, thr);
    }

    //TODO: do it more accurately? but its fast
    bool extensionMatch(const ex_curved_polygon &o, const float thr, const float var) const
    {
      const float l1 = bb_.extension();
      const float l2 = o.bb_.extension();

      return std::max(std::abs(l1-l2)-var,0.f) < std::min(l1,l2) * thr;
    }

    bool merge(const ex_curved_polygon &o1, const ex_curved_polygon &o2) {
      //      PolygonData p1, p2, p3;
      //
      //      for(size_t i=0; i<o1.outline_.segments_.size(); i++)
      //      {
      //        //if(o1.segments_[i](2)>0.5f)
      //        {
      //          Eigen::Vector2f p = surface_->nextPoint(o1.surface_->project2world(o1.outline_.segments_[i].head<2>() ));
      //          p1.add(p(0), p(1));
      //          //          std::cout<<"p1\n"<<p<<"\n";
      //          //          std::cout<<"s1\n"<<o1.segments_[i]<<"\n";
      //          //          std::cout<<"w1\n"<<surface_->project2world(p )<<"\n";
      //        }
      //      }
      //      std::cout<<"PARAMS\n"<<proj2plane_<<"\n";
      //      std::cout<<param_<<"\n";
      //      for(size_t i=0; i<o2.outline_.segments_.size(); i++)
      //      {
      //        //if(o2.segments_[i](2)>0.5f)
      //        {
      //          Eigen::Vector2f p = surface_->nextPoint(o2.surface_->project2world(o2.outline_.segments_[i].head<2>() ));
      //          p2.add(p(0), p(1));
      //          //          std::cout<<"p2\n"<<p<<"\n";
      //          //          std::cout<<"s2\n"<<o2.segments_[i]<<"\n";
      //          //          std::cout<<"w2\n"<<surface_->project2world(p )<<"\n";
      //        }
      //      }
      //
      //      mergePolygons(p1,p2, p3);
      //
      //      if(p3.get().size()<=0)
      //        return false;

      Outline p1, p2;

      p1.weight_ = std::pow(o1.data_.weight,0.5f);
      p2.weight_ = std::pow(o2.data_.weight,0.5f);

      for(size_t i=0; i<o1.outline_.segments_.size(); i++)
      {
        //if(o1.segments_[i](2)>0.5f)
        {
          Eigen::Vector3f p;
          p.head<2>() = surface_->nextPoint(o1.surface_->project2world(o1.outline_.segments_[i].head<2>() ));
          p(2) = p1.weight_*o1.outline_.segments_[i](2);
          p1 += p;

          //          ROS_INFO("dist %f", (o1.surface_->project2world(o1.outline_.segments_[i].head<2>())-surface_->project2world(p.head<2>())).norm() );
        }
      }

      for(size_t i=0; i<o2.outline_.segments_.size(); i++)
      {
        //if(o2.segments_[i](2)>0.5f)
        {
          Eigen::Vector3f p;
          p.head<2>() = surface_->nextPoint(o2.surface_->project2world(o2.outline_.segments_[i].head<2>() ));
          p(2) = p2.weight_*o2.outline_.segments_[i](2);
          p2 += p;

          //          ROS_INFO("dist %f", (o2.surface_->project2world(o2.outline_.segments_[i].head<2>())-surface_->project2world(p.head<2>())).norm() );
        }
      }

      static int cnt=0;
      ++cnt;
      char buffer[128];
      //      sprintf(buffer,"%dA.svg",cnt);
      //      p1.debug_svg(buffer);
      //      sprintf(buffer,"%dB.svg",cnt);
      //      p2.debug_svg(buffer);

      //      p1.reverse(); //TODO: change alog. instead of this
      //      p2.reverse();
      outline_ = p1+p2;
      //      outline_.reverse();

      //      for(size_t i=0; i<outline_.segments_.size(); i++)
      //        outline_.segments_[i](2) /= std::max(o1.data_.weight, o2.data_.weight);

#ifdef DEBUG_SVG
      sprintf(buffer,"%f %dC.svg",Debug::Interface::get().getTime(), cnt);
      p1.debug_svg(p2.debug_svg(outline_.debug_svg(buffer,200,500,"red"),200,500,"green"),200,500);
#endif

      Classification::Classification::MainPoints compressed;
      Classification::Classification::MainPoints mains;
      mains.size_ = 4;
      Classification::Classification cl;
      Classification::QuadBB qbb;
      cl.setPoints(outline_.segments_);
      form_.n_ = cl.buildLocalTree(false,&qbb,&compressed,NULL,&mains);
      if(form_.n_)
        form_.n_->clean(1,form_.n_->energy_sum_*0.01f);

#ifdef DEBUG_OUTPUT_
      ROS_INFO("COMPRESSION %d %d",data_.polyline.size(),compressed.points_.size());
#endif

      data_.polyline.clear();

      for(size_t i=0; i<compressed.points_.size(); i++)
      {
        cob_3d_mapping_msgs::polyline_point pt;
        pt.x = outline_.segments_[compressed.points_[i].index_](0);
        pt.y = outline_.segments_[compressed.points_[i].index_](1);
        pt.edge_prob = compressed.points_[i].edge_;
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
    bool operator+=(ex_curved_polygon o) {
      int st;
      return op_plus(o,st);
    }

    bool op_plus(ex_curved_polygon o, int &status) {
      if(o.outline_.segments_.size()<1 || outline_.segments_.size()<1) {
        status=0;
        return false;
      }

      //merge parameters to third manifold
      ex_curved_polygon o2 = *this;

      bool sw = false;
      if(!canMerge(o, 0.1f, &sw)) {
#ifdef DEBUG_OUT_
        ROS_INFO("could not merge1");
#endif
        status=1;
        return false;
      }

      //      if(sw)
      //      {
      //        surface_ = o.surface_;
      //        data_.parameter = o.data_.parameter;
      //      }

      //combine outline
      Slam_Surface::Surface::SWINDOW w1 = getWindow(), w2 = o.getWindow();
      if(sw)
      {
        //        SURFACE s = o.surface_;
        //        float dist = s.merge(surface_, o.data_.weight, data_.weight,w2,w1);
        //        if(dist > 0.03f + 0.03f*getNearestPoint().squaredNorm()) {
        //          ROS_INFO("could not merge %f",dist);
        //          //return false;
        //        }
        float dist;
        surface_.reset( new SURFACE(*o.surface_) );
        if((dist=surface_->merge(*o2.surface_, o.data_.weight, data_.weight,w2,w1)) > 0.03f + 0.05f*std::max(1.f,getNearestPoint().squaredNorm())) {
          surface_.reset( new SURFACE(*o2.surface_) );
#ifndef DEBUG_OUT_
          ROS_INFO("could not merge2 %f",dist);
#endif
          status=2;
          return false;
        }

#ifdef DEBUG_
        Debug::Interface::get().addArrow(o2.getNearestPoint(), o.getNearestPoint(), 255,0,0);
#endif
      }
      else {
        //        SURFACE s = surface_;
        //        float dist = s.merge(o.surface_, data_.weight, o.data_.weight,w1,w2);
        //        if(dist > 0.03f + 0.03f*getNearestPoint().squaredNorm()) {
        //          ROS_INFO("could not merge %f",dist);
        //          //return false;
        //        }
        float dist;
        if((dist=surface_->merge(*o.surface_, data_.weight, o.data_.weight,w1,w2)) > 0.03f + 0.05f*std::max(1.f,getNearestPoint().squaredNorm())) {
          surface_.reset( new SURFACE(*o2.surface_) );
#ifndef DEBUG_OUT_
          ROS_INFO("could not merge3 %f",dist);
#endif
          status=2;
          return false;
        }

#ifdef DEBUG_
        Debug::Interface::get().addArrow(o2.getNearestPoint(), o.getNearestPoint(), 0,0,255);
#endif
      }

      if(!merge(o,o2))
      {
        //        surface_ = o2.surface_;
        //        data_.parameter = o2.data_.parameter;
#ifdef DEBUG_OUT_
        ROS_INFO("could not merge4");
#endif
        status=3;
        return false;
      }

      color_.r = (data_.weight*color_.r+o.data_.weight*o.color_.r)/(data_.weight + o.data_.weight);
      color_.g = (data_.weight*color_.g+o.data_.weight*o.color_.g)/(data_.weight + o.data_.weight);
      color_.b = (data_.weight*color_.b+o.data_.weight*o.color_.b)/(data_.weight + o.data_.weight);

      ID_ += o.ID_;

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

#ifdef DEBUG_OUT_
      ROS_INFO("match %f %f",e,0.2f*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_));
#endif

      return e<0.4f*std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_);
    }

    float matchFormf(const ex_curved_polygon &o) const {
      if(!form_.n_ || ! o.form_.n_)
        return false;

      return std::max(0.2f,1.2f-form_.n_->search(*o.form_.n_, 0.8f)/std::min(form_.n_->energy_sum_, o.form_.n_->energy_sum_));
    }

    float getEnergy() const {return form_.n_->energy_;}
    float getWeight() const {return data_.weight;}
    void printEnergy() const {form_.n_->print();}

    void triangle(std::vector<Eigen::Vector3f> &tri, const TPPLPoint &p1, const TPPLPoint &p2, const TPPLPoint &p3, const int depth=0) const {
      if(depth>1)
      {
        Eigen::Vector2f p;
        p(0) = p1.x;
        p(1) = p1.y;
        tri.push_back(surface_->project2world(p));
        p(0) = p2.x;
        p(1) = p2.y;
        tri.push_back(surface_->project2world(p));
        p(0) = p3.x;
        p(1) = p3.y;
        tri.push_back(surface_->project2world(p));
      }
      else
      {
        TPPLPoint p4, p12,p23,p31;

        p4.x = (p1.x+p2.x+p3.x)/3;
        p4.y = (p1.y+p2.y+p3.y)/3;
        p12.x = (p1.x+p2.x)/2;
        p12.y = (p1.y+p2.y)/2;
        p23.x = (p3.x+p2.x)/2;
        p23.y = (p3.y+p2.y)/2;
        p31.x = (p1.x+p3.x)/2;
        p31.y = (p1.y+p3.y)/2;

        triangle(tri,p1,p12,p4,depth+1);
        triangle(tri,p1,p31,p4,depth+1);
        triangle(tri,p3,p23,p4,depth+1);

        triangle(tri,p12,p2,p4,depth+1);
        triangle(tri,p31,p3,p4,depth+1);
        triangle(tri,p23,p2,p4,depth+1);
      }
    }

    void getTriangles(std::vector<Eigen::Vector3f> &tri) const
    {
      //      for(size_t i=0; i<features_.size(); i++)
      //      {
      //        if(features_[i].ID_!=-1 || !pcl_isfinite(features_[2].v_.sum()) || !pcl_isfinite(features_[i].v_.sum())) continue;
      //        if(tri.size()>2)
      //          tri.push_back( tri[tri.size()-1] );
      //        else
      //          tri.push_back( features_[i].v_ );
      //        tri.push_back( features_[2].v_ );
      //        tri.push_back( features_[i].v_ );
      //      }
      //      if(tri.size()>2)
      //        tri[0] = tri[tri.size()-1];

      //triangulate

      TPPLPartition pp;
      list<TPPLPoly> polys,result;

      //fill polys
      TPPLPoly poly;
      poly.Init(outline_.size());
      poly.SetHole(false);

      for(size_t i=0; i<outline_.size(); i++) {
        TPPLPoint p;
        p.x = outline_[i](0);
        p.y = outline_[i](1);
        poly[i] = p;
      }
      poly.SetOrientation(TPPL_CCW);

      polys.push_back(poly);

      pp.Triangulate_EC(&polys,&result);

      TPPLPoint p1,p2,p3,p4, p12,p23,p31;

      for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
        //draw each triangle
        if(it->GetNumPoints()!=3) continue;

        p1 = it->GetPoint(0);
        p2 = it->GetPoint(1);
        p3 = it->GetPoint(2);

        triangle(tri,p1,p2,p3);
      }
    }

    void getControlPoints(std::vector<std::vector<Eigen::Vector3f> > &pts) const
    {
#ifdef SURFACE_NURBS
      for(int i=0; i<surface_->getNurbs().ctrlPnts().rows(); i++) {
        pts.push_back(std::vector<Eigen::Vector3f>());
        for(int j=0; j<surface_->getNurbs().ctrlPnts().cols(); j++) {
          Eigen::Vector2f v2;
          v2(0)=i/(float)(surface_->getNurbs().ctrlPnts().rows()-1);
          v2(1)=j/(float)(surface_->getNurbs().ctrlPnts().cols()-1);
          pts.back().push_back(surface_->_project2world(v2));
        }
      }
#endif
    }

    bool canMerge(const ex_curved_polygon &o, const float diff=0.001f, bool *sw=NULL) const {

      PolygonData p1, p2, p3;

      for(size_t i=0; i<o.outline_.segments_.size(); i++)
      {
        Eigen::Vector2f p = surface_->nextPoint(o.surface_->project2world(o.outline_.segments_[i].head<2>() ));
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

      const float rel = unionPolygons(p1,p2, p3, sw);
      if(rel<0.1f)
      {
#ifndef DEBUG_OUT_
        ROS_INFO("merge break 1 %f",rel);
#endif
        return false;
      }

      float lges=0.f, er=0.f, erlast=0.f, lfirst=0.f;
      float er2=0;
      int bad_ctr = 0;

      for(size_t i=0; i<p3.get().size()/2; i++)
      {
        Eigen::Vector2f v1, v3;
        Eigen::Vector3f t, n, v2, normal;
        v1(0)=p3.get()[i*2+0];
        v1(1)=p3.get()[i*2+1];
        //        v2(0) = ((v1(0)*proj2plane_(0,0)+param_.col(0)(0))-o.param_.col(0)(0))/o.proj2plane_(0,0);
        //        v2(1) = ((v1(1)*proj2plane_(1,1)+param_.col(0)(1))-o.param_.col(0)(1))/o.proj2plane_(1,1);

        t = surface_->project2world(v1);
        Eigen::Vector2f np = o.surface_->nextPoint(t);
        v2 = o.surface_->project2world(np);
        normal = surface_->normalAt(v1);

        v1(0)=p3.get()[(i*2+2)%p3.get().size()];
        v1(1)=p3.get()[(i*2+3)%p3.get().size()];
        n = surface_->project2world(v1);

        if(
            //(t-v2).norm() > 0.03f+diff*getNearestPoint().squaredNorm() ||
            o.surface_->normalAt(np).dot(normal) < 0.9f
        )
        {
          ++bad_ctr;
          //if(bad_ctr>1)
          {
#ifndef DEBUG_OUT_
            ROS_INFO("merge break 2 %f",o.surface_->normalAt(np).dot(normal));
#endif
            return false;
          }
        }

        float dist2 = 1/(std::max(0.1f,  o.surface_->normalAt(np).dot(normal) ) + 0.1f);

        const float l = (t-n).norm();
        er += l*erlast;
        erlast = /*(t-v2).norm()*/ (v2-t).norm() *dist2 /(1+((t+n)*0.5f).squaredNorm());
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
      //        t = surface_->project2world(v1);
      //        v2 = o.surface_->project2world(o.surface_->nextPoint(t));
      //
      //        v1=p3.segments_[(i+1)%p3.segments_.size()].head<2>();
      //        n = surface_->project2world(v1);
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

#ifndef DEBUG_OUTPUT_
      if(std::abs(er)/lges>=0.03f+diff)
      ROS_INFO("merge break proj %f", er/lges);
#endif

      return std::abs(er)/lges<0.03f+diff;
    }

    bool invalid() const
    {
      if(!pcl_isfinite(getNearestPoint().sum())) {
        ROS_ERROR("rm3");
        return true;
      }

      if(data_.polyline.size()<3)
      {
        ROS_ERROR("rm4");
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

      if(!outline_.check())
        return true;

      return false;
    }

    inline const Outline &getOutline() const {return outline_;}
    inline const SURFACE &getSurface() const {return *surface_;}

    const ::std_msgs::ColorRGBA &getColor() const {return color_;}
    void setColor(const ::std_msgs::ColorRGBA &c) {color_=c;}

    const std::vector<Eigen::Vector3f> &getPoints3D() const {return points3d_;}
  };

}


#endif /* OBJECT_H_ */
