/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: dynamic_tutorials
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: josh
 *
 * Date of creation: Oct 26, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef SEGMENTATION_QUAD_REGR_H_
#define SEGMENTATION_QUAD_REGR_H_

#include <visualization_msgs/MarkerArray.h>
#include "quad_regression_algo.h"

namespace Segmentation
{


  //example for parent: QPPF::QuadRegression<QPPF::Degree2, Point, QPPF::CameraModel_Kinect<Point> >

  /**
   * a segmentation implementation based on quad-trees and regression
   */
  template <typename Point, typename PointLabel, typename Parent>
  class Segmentation_QuadRegression : public GeneralSegmentation<Point, PointLabel>, public Parent
  {

    void back_check_repeat(); /// repeat back check on model

    boost::shared_ptr<const pcl::PointCloud<PointLabel> > compute_labeled_pc();
    boost::shared_ptr<const pcl::PointCloud<PointLabel> > compute_reconstructed_pc();

  public:
    /// destructor
    virtual ~Segmentation_QuadRegression() {
    }

    /// gets preprocessed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getOutputCloud() {
      return compute_labeled_pc();
    }

    /// gets reconstructed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getReconstructedOutputCloud() {
      return compute_reconstructed_pc();
    }

    /// sets preprocessed input cloud
    virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
    {
      this->Parent::setInputCloud(cloud);
    }

    virtual bool compute();

    virtual bool extractImages();

    /// convert to ROS message
    operator cob_3d_mapping_msgs::ShapeArray() const;

    /// convert edges to ROS message
    operator visualization_msgs::Marker() const;

    /*** evaluation purposes ***/
    void compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive);

  };

#include "impl/quad_regression.hpp"

#if 0

#define getInd(x, y) ((x)+(y)*levels_[i].w)
#define getInd1(x, y) ((x)+(y)*levels_[i-1].w)
#define getInd2(x, y) ((x)+(y)*levels_[i+2].w)
#define getInd3(x, y) ((x)+(y)*levels_[i+1].w)
#define getIndPC(x, y) ((x)+(y)*pc.width)

  /**
   * a segmentation implementation based on quad-trees and regression
   */
  template <typename Point, typename PointLabel>
  class Segmentation_QuadRegression : public GeneralSegmentation<Point, PointLabel>
  {
    //------------STRUCTURES-----------

    /**
     * stores parameters of camera (kinect, xtion, ...)
     */
    struct SKINECTPARAMS {
      float f,dx,dy;
    };

    //-----------CONSTANTS-------------
    const unsigned int MIN_LOD; /// minimum of nodes for a segment
    const unsigned int FINAL_LOD; /// lowest level = 0
    const unsigned int GO_DOWN_TO_LVL; /// search down to ...

    //------------MEMBERS---------------
    boost::shared_ptr<const pcl::PointCloud<Point> > input_;

    std::vector<SubStructure::ParamC> levels_; ///quad-tree
    std::vector<Segmentation::S_POLYGON> polygons_;
    int *ch_; /// mark-array
    SKINECTPARAMS kinect_params_; ///camera parameters

    bool *outline_check_;         ///needed for outline, no need to reallocate every time
    size_t outline_check_size_;    ///remember size for var. above
    float filter_;                /// ratio points per area
    bool only_planes_;            /// filter option

#ifdef STOP_TIME
    double execution_time_quadtree_, execution_time_growing_, execution_time_polyextraction_;
#endif

    void prepare(const pcl::PointCloud<Point> &pc); /// setup level size
    void getKinectParams(const pcl::PointCloud<Point> &pc); /// calculate kinect parameters

    void buildTree(const pcl::PointCloud<Point> &pc); ///build quad-tree
    void calc(); /// segmentation on quad-tree
    void back_check_repeat(); /// repeat back check on model

    void grow(SubStructure::Model &model, const int i, const int x, const int y);
    void grow(SubStructure::VISITED_LIST<SubStructure::SVALUE> &list, SubStructure::Model &model, const int i, const int mark, bool first_lvl);

    inline bool filterOccupied(const int i, const int x, const int y, const int mark) const {
      if(i>=(int)levels_.size())
        return false;
      if(levels_[i].data[getInd(x,y)].occopied==mark)
        return true;
      return filterOccupied(i+1,x/2,y/2,mark);
    }

    inline int otherOccupied(const int i, const int x, const int y, const int mark) const {
      if(i>=(int)levels_.size() || x<0 || y<0 || x>=(int)levels_[i].w || y>=(int)levels_[i].h)
        return -1;
      if(levels_[i].data[getInd(x,y)].occopied>=0 && levels_[i].data[getInd(x,y)].occopied!=mark)
        return levels_[i].data[getInd(x,y)].occopied;
      return otherOccupied(i+1,x/2,y/2,mark);
    }

    inline int isOccupied(const int i, const int x, const int y) const {
      if(i>=(int)levels_.size())
        return -1;
      if(levels_[i].data[getInd(x,y)].occopied>=0)
        return levels_[i].data[getInd(x,y)].occopied;
      return isOccupied(i+1,x/2,y/2);
    }

    inline bool checkModelAt(const SubStructure::Model &model, const int i, const int x, const int y, const float thr) const {
      if(i==0)
        return model.check_tangent(levels_[i].data[getInd(x,y)], thr);

      return //model.check(levels_[i].data[getInd(x,y)], thr) &&
          model.check_tangent(levels_[i-1].data[getInd1(2*x,2*y)], thr) &&
          model.check_tangent(levels_[i-1].data[getInd1(2*x,2*y+1)], thr) &&
          model.check_tangent(levels_[i-1].data[getInd1(2*x+1,2*y)], thr) &&
          model.check_tangent(levels_[i-1].data[getInd1(2*x+1,2*y+1)], thr);
    }

    inline void addPoint(const int i, const int x, const int y, const int mark, S_POLYGON &poly, const SubStructure::Model &model, const float v=0.f) {

#if 0
      for(int xx=-1; xx<2; xx++)
        for(int yy=-1; yy<2; yy++)
          if(x+xx>=0 && y+yy>=0 && x+xx<(int)levels_[i].w && y+yy<(int)levels_[i].h
              && filterOccupied(i,x+xx,y+yy,mark) //&& (levels_[i].data[getInd(x+xx,y+yy)].occopied==mark||levels_[i+1].data[getInd3((x+xx)/2,(y+yy)/2)].occopied==mark)
      //&& poly.model_.check_tangent(levels_[i].data[getInd(x+xx,y+yy)],0.02f)
          ){
            Eigen::Vector3f p;
            p(0) = levels_[i].data[getInd(x+xx,y+yy)].model_(0,1)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
            p(1) = levels_[i].data[getInd(x+xx,y+yy)].model_(0,3)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
            p(2) = levels_[i].data[getInd(x+xx,y+yy)].z_(0)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);


            //            Eigen::Vector3f p = poly.project2plane(levels_[i].data[getInd(x+xx,y+yy)].model_(0,1)/levels_[i].data[getInd(x+xx,y+yy)].z_(0),
            //                                                   levels_[i].data[getInd(x+xx,y+yy)].model_(0,3)/levels_[i].data[getInd(x+xx,y+yy)].z_(0),
            //                                                   model,v);

            p.head<2>() = poly.nextPoint(p);
            /*
Eigen::Vector2f vv;
      vv(0) = (p-poly.param_.col(0)).dot(poly.proj2plane_.col(0))/poly.proj2plane_.col(0).squaredNorm();
      vv(1) = (p-poly.param_.col(0)).dot(poly.proj2plane_.col(1))/poly.proj2plane_.col(1).squaredNorm();
            p.head<2>() = vv;*/

            if(!pcl_isfinite(p.sum())) continue;
            p(2) = v;
            poly.segments_.back().push_back(p);
            return;
          }
      for(int xx=-1; xx<2; xx++)
        for(int yy=-1; yy<2; yy++)
          if(x+xx>=0 && y+yy>=0 && x+xx<(int)levels_[i].w && y+yy<(int)levels_[i].h
              && poly.model_.check_tangent(levels_[i].data[getInd(x+xx,y+yy)],0.02f)){
            Eigen::Vector3f p;
            p(0) = levels_[i].data[getInd(x+xx,y+yy)].model_(0,1)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
            p(1) = levels_[i].data[getInd(x+xx,y+yy)].model_(0,3)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
            p(2) = levels_[i].data[getInd(x+xx,y+yy)].z_(0)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
            /*            p.head<2>() = poly.nextPoint(p);

Eigen::Vector2f vv;
      vv(0) = (p-poly.param_.col(0)).dot(poly.proj2plane_.col(0))/poly.proj2plane_.col(0).squaredNorm();
      vv(1) = (p-poly.param_.col(0)).dot(poly.proj2plane_.col(1))/poly.proj2plane_.col(1).squaredNorm();
            p.head<2>() = vv;
             */
            if(!pcl_isfinite(p.sum())) continue;
            p(2) = v;
            poly.segments_.back().push_back(p);
            return;
          }

      //      for(int xx=-2; xx<3; xx++)
      //        for(int yy=-2; yy<3; yy++)
      //          if(x+xx>=0 && y+yy>=0 && x+xx<levels_[i].w && y+yy<levels_[i].h && levels_[i].data[getInd(x+xx,y+yy)].occopied==mark){
      //            Eigen::Vector3f p;
      //            p(0) = levels_[i].data[getInd(x+xx,y+yy)].model_(0,1)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
      //            p(1) = levels_[i].data[getInd(x+xx,y+yy)].model_(0,3)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
      //            p(2) = levels_[i].data[getInd(x+xx,y+yy)].z_(0)/levels_[i].data[getInd(x+xx,y+yy)].model_(0,0);
      //            //p.head<2>() = poly.nextPoint(p);
      //
      //            p.head<2>() -= poly.getCenter();
      //            p(0)/=poly.proj2plane_(0,0);
      //            p(1)/=poly.proj2plane_(1,1);
      //
      //            if(!pcl_isfinite(p.sum())) continue;
      //            p(2) = v;
      //            poly.segments_.back().push_back(p);
      //            return;
      //          }
#endif

#ifdef DO_NOT_DOWNSAMPLE_
      const int fact=1;
#else
      const int fact=2;
#endif

      /*{
        Eigen::Vector3f p = poly.project2plane(((x<<(i+fact-1))-kinect_params_.dx)/kinect_params_.f,
                                               ((y<<(i+fact-1))-kinect_params_.dy)/kinect_params_.f,
                                               model,v);
      poly.segments_.back().push_back(p);
#ifdef USE_BOOST_POLYGONS_
poly.segments2d_.back().push_back(BoostPoint(x,y));
#elif defined(BACK_CHECK_REPEAT)
Eigen::Vector2i p2;
p2(0) = x;
p2(1) = y;
poly.segments2d_.back().push_back(p2);
#endif
      }
      return;*/

      Eigen::Vector3f p = poly.project2plane(((x<<(i+fact-1))-kinect_params_.dx)/kinect_params_.f,
                                             ((y<<(i+fact-1))-kinect_params_.dy)/kinect_params_.f,
                                             model,v);

#if 0
      poly.segments_.back().push_back(p);
#ifdef USE_BOOST_POLYGONS_
      poly.segments2d_.back().push_back(BoostPoint(x,y));
#elif defined(BACK_CHECK_REPEAT)
      Eigen::Vector2i p2;
      p2(0) = x;
      p2(1) = y;
      poly.segments2d_.back().push_back(p2);
#endif

      return;
#endif

      Eigen::Vector3f vp = poly.project2world(p.head<2>());

      float d = poly.middle_(2);
      d*=d;

      static const int rel_motion_pattern[][2] = { {0,0}, {-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}
      //, {-2,-2}, {-2,-1}, {-2,0}, {-2,1}, {-2,2}, {-1,-2}, {-1,2}, {0,-2}, {0,2}, {1,-2}, {1,2}, {2,-2}, {2,-1}, {2,0}, {2,1}, {2,2}
      //, {-3,-3}, {-3,-2}, {-3,-1}, {-3,0}, {-3,1}, {-3,2}, {-3,3}, {-2,-3}, {-2,3}, {-1,-3}, {-1,3}, {0,-3}, {0,3}, {1,-3}, {1,3}, {2,-3}, {2,3}, {3,-3}, {3,-2}, {3,-1}, {3,0}, {3,1}, {3,2}, {3,3}, {-4,-4}, {-4,-3}, {-4,-2}, {-4,-1}, {-4,0}, {-4,1}, {-4,2}, {-4,3}, {-4,4}, {-3,-4}, {-3,4}, {-2,-4}, {-2,4}, {-1,-4}, {-1,4}, {0,-4}, {0,4}, {1,-4}, {1,4}, {2,-4}, {2,4}, {3,-4}, {3,4}, {4,-4}, {4,-3}, {4,-2}, {4,-1}, {4,0}, {4,1}, {4,2}, {4,3}, {4,4}
      };


      const pcl::PointCloud<Point> &pc = *input_;
      //      for(int xx=-4; xx<5; xx++)
      //        for(int yy=-4; yy<5; yy++)
      for(size_t i=0; i<sizeof(rel_motion_pattern)/sizeof(rel_motion_pattern[0]); i++) {
        int xx=rel_motion_pattern[i][0];
        int yy=rel_motion_pattern[i][1];

        if(fact*x+xx>=0 && fact*y+yy>=0 && fact*x+xx<(int)pc.width && fact*y+yy<(int)pc.height) {
          Eigen::Vector3f p = poly.project2plane((((fact*x+xx)<<(i))-kinect_params_.dx)/kinect_params_.f,
                                                 (((fact*y+yy)<<(i))-kinect_params_.dy)/kinect_params_.f,
                                                 model,v);

          if( (poly.project2world(p.head<2>())-pc[getIndPC(fact*x+xx,fact*y+yy)].getVector3fMap()).squaredNorm()<std::max(0.0005f,0.0005f*d)
          && (poly.project2world(p.head<2>())-vp).squaredNorm()<0.02f)
          {
            poly.segments_.back().push_back(p);/*
          Eigen::Vector2f p = poly.nextPoint(pc[getIndPC(fact*x+xx,fact*y+yy)].getVector3fMap());

          if( (poly.project2world(p)-pc[getIndPC(fact*x+xx,fact*y+yy)].getVector3fMap()).squaredNorm()<std::max(0.0005f,0.0005f*d)
          && (poly.project2world(p)-vp).squaredNorm()<0.02f
          )
          {
            Eigen::Vector3f p3;
            p3.head<2>()=p;
            poly.segments_.back().push_back(p3);*/

            //            std::cout<<"P\n"<<poly.project2world(p.head<2>())<<"\n";
            //            std::cout<<"C\n"<<pc[getIndPC(2*x+xx,2*y+yy)].getVector3fMap()<<"\n";

#ifdef USE_BOOST_POLYGONS_
            poly.segments2d_.back().push_back(BoostPoint(x,y));
#elif defined(BACK_CHECK_REPEAT)
            Eigen::Vector2i p2;
            p2(0) = x+xx/fact;
            p2(1) = y+yy/fact;
            poly.segments2d_.back().push_back(p2);
#endif

            return;
          }
        }

      }
#if 0
      for(int xx=-8; xx<9; xx++)
        for(int yy=-8; yy<9; yy++)
          if(fact*x+xx>=0 && fact*y+yy>=0 && fact*x+xx<(int)pc.width && fact*y+yy<(int)pc.height) {
            Eigen::Vector3f p = poly.project2plane((((fact*x+xx)<<(i))-kinect_params_.dx)/kinect_params_.f,
                                                   (((fact*y+yy)<<(i))-kinect_params_.dy)/kinect_params_.f,
                                                   model,v);

            if( (poly.project2world(p.head<2>())-pc[getIndPC(fact*x+xx,fact*y+yy)].getVector3fMap()).squaredNorm()<std::max(0.0005f,0.0005f*d)
            && (poly.project2world(p.head<2>())-vp).squaredNorm()<0.02f)
            {
              poly.segments_.back().push_back(p);

              //            std::cout<<"P\n"<<poly.project2world(p.head<2>())<<"\n";
              //            std::cout<<"C\n"<<pc[getIndPC(2*x+xx,2*y+yy)].getVector3fMap()<<"\n";

              //            std::cout<<"P\n"<<poly.project2world(p.head<2>())<<"\n";
              //            std::cout<<"C\n"<<pc[getIndPC(2*x+xx,2*y+yy)].getVector3fMap()<<"\n";

#ifdef USE_BOOST_POLYGONS_
              poly.segments2d_.back().push_back(BoostPoint(x,y));
#elif defined(BACK_CHECK_REPEAT)
              Eigen::Vector2i p2;
              p2(0) = x+xx/fact;
              p2(1) = y+yy/fact;
              poly.segments2d_.back().push_back(p2);
#endif

              return;
            }
          }
#endif

      if(poly.normalAt(p.head<2>())(2)>0.5f)
      {
        poly.segments_.back().push_back(p);
#ifdef USE_BOOST_POLYGONS_
        poly.segments2d_.back().push_back(BoostPoint(x,y));
#elif defined(BACK_CHECK_REPEAT)
        Eigen::Vector2i p2;
        p2(0) = x;
        p2(1) = y;
        poly.segments2d_.back().push_back(p2);
#endif
      }
    }

    int getPos(int *ch, const int xx, const int yy, const int w, const int h);

    void outline(int *ch, const int w, const int h, std::vector<SubStructure::SXY> &out, const int i, S_POLYGON &poly, const SubStructure::Model &model, const int mark);

    boost::shared_ptr<const pcl::PointCloud<PointLabel> > compute_labeled_pc();
    boost::shared_ptr<const pcl::PointCloud<PointLabel> > compute_reconstructed_pc();

  public:
    /// constructor, setups variables
    Segmentation_QuadRegression();

    /// destructor
    virtual ~Segmentation_QuadRegression() {
      delete [] ch_;
      delete [] outline_check_;
    }

    /// sets preprocessed input cloud
    virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
    {
      input_ = cloud;
      if(levels_.size()==0)
        prepare(*cloud);
      else if(kinect_params_.f == 0.f)
        getKinectParams(*cloud);
    }

    /// gets preprocessed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getOutputCloud() {
      return compute_labeled_pc();
    }

    /// gets reconstructed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getReconstructedOutputCloud() {
      return compute_reconstructed_pc();
    }

    virtual bool compute();

    virtual bool extractImages();

    /// convert to ROS message
    operator cob_3d_mapping_msgs::ShapeArray() const;

    /// convert edges to ROS message
    operator visualization_msgs::Marker() const;

    /// get polygons
    const std::vector<Segmentation::S_POLYGON> &getPolygons() const {return polygons_;}

    /*** evaluation purposes ***/
    void compute_accuracy(float &mean, float &var, size_t &used, size_t &mem, size_t &points, float &avg_dist);

#ifdef STOP_TIME
    void getExecutionTimes(double &quadtree, double &growing, double &extraction)
    {
      quadtree = execution_time_quadtree_;
      extraction = execution_time_polyextraction_;
      growing = execution_time_growing_ - extraction;
    }
#endif

    void setFilter(const float f) {filter_=f;}
    void setOnlyPlanes(const bool b) {only_planes_=b;}

  };

#include "impl/quad_regression.hpp"

#undef getInd
#undef getInd1
#undef getInd2
#undef getIndPC
#endif
}

#endif /* SEGMENTATION_QUAD_REGR_H_ */
