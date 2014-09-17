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

#ifndef QUAD_REGRESSION_ALGO_H_
#define QUAD_REGRESSION_ALGO_H_

#include <pcl/kdtree/kdtree_flann.h>
#include "../general_segmentation.h"

#define USE_MIN_MAX_RECHECK_
#define STOP_TIME
//#define BACK_CHECK_REPEAT
#define DO_NOT_DOWNSAMPLE_
//#define SIMULATION_
//#define MIN_EIGEN_VECTOR
//#define EVALUATE
//#define USE_BOOST_POLYGONS_

#include "polygon.h"

namespace Segmentation
{

  namespace QPPF
  {

    /**
     * camera model for Kinect
     *
     *  - extracts parameters from pc
     *  - states standard deviation for a given distance
     */
    template<typename Point>
    class CameraModel_Kinect {
    public:
      float f,dx,dy;

      CameraModel_Kinect(): f(0.f) {}

      /// calculate kinect parameters
      void getParams(const pcl::PointCloud<Point> &pc);

      inline static float std(const float dist) {return 0.0075f+0.0020425f*dist*dist;}   //after "Accuracy analysis of kinect depth data"

      /** DEBUG **/
      template<typename APoint>
      friend std::ostream &operator<<(ostream &os, const CameraModel_Kinect<APoint> &cmk);
    };

    /**
     * camera model for SR4500
     *
     *  - extracts parameters from pc
     *  - states standard deviation for a given distance
     */
    template<typename Point>
    class CameraModel_SR4500 {
    public:
      float f,dx,dy;

      CameraModel_SR4500(): f(0.f) {}

      /// calculate kinect parameters
      void getParams(const pcl::PointCloud<Point> &pc);

      inline static float std(const float dist) {return 0.00015f+0.00015f*dist;}   //after "Accuracy analysis of kinect depth data"

      /** DEBUG **/
      template<typename APoint>
      friend std::ostream &operator<<(ostream &os, const CameraModel_Kinect<APoint> &cmk);
    };

    template<typename Point>
    std::ostream &operator<<(ostream &os, const CameraModel_Kinect<Point> &cmk) {
      os<<"Kinect Camera\n";
      os<<"f = "<<cmk.f<<std::endl;
      os<<"dx = "<<cmk.dx<<std::endl;
      os<<"dy = "<<cmk.dy<<std::endl;
      return os;
    }

    /**
     * a segmentation implementation based on quad-trees and regression
     */
    template <int Degree, typename Point, typename CameraModel>
    class QuadRegression
    {
    protected:

      /// get index for element at (x,y) on level i (0: leaves) of quadtree
      inline int getInd(const int i, const int x, const int y) const {
        return (x)+(y)*levels_[i].w;
      }

      /// get index for element at (x,y) in ordered pc
      inline int getIndPC(const int w, const int x, const int y) const {
        return (x)+(y)*w;
      }

      /// check occupation from down to top
      inline int isOccupied(const int i, const int x, const int y) const {
        if(i>=(int)levels_.size())
          return -1;
        if(levels_[i].data[getInd(i,x,y)].occopied>=0)
          return levels_[i].data[getInd(i,x,y)].occopied;
        return isOccupied(i+1,x/2,y/2);
      }

      //------------MEMBERS---------------
      boost::shared_ptr<const pcl::PointCloud<Point> > input_;          ///input point cloud (no need for color)
      std::vector<SubStructure::ParamC<Degree> > levels_;               ///quad-tree levels/stages
      std::vector<Segmentation::S_POLYGON<Degree> > polygons_;          ///result of surface reconstruction

#ifdef STOP_TIME
      // evaluation variables
      double execution_time_quadtree_, execution_time_growing_, execution_time_polyextraction_;
#endif

      enum {DEGREE=Degree};             // constant for degree (1=linear)

    private:
      //-----------CONSTANTS-------------
      const unsigned int MIN_LOD;       /// minimum of nodes for a segment
      const unsigned int FINAL_LOD;     /// lowest level = 0
      const unsigned int GO_DOWN_TO_LVL;/// search down to ...

      //------------MEMBERS---------------
      CameraModel camera_;              /// camera model

      int *ch_; /// mark-array

      bool *outline_check_;             /// needed for outline, no need to reallocate every time
      size_t outline_check_size_;       /// remember size for var. above
      float filter_;                    /// ratio points per area
      bool only_planes_;                /// filter option

      void grow(SubStructure::Model<Degree> &model, const int i, const int x, const int y);
      void grow(SubStructure::VISITED_LIST<SubStructure::SVALUE> &list, SubStructure::Model<Degree> &model, const int i, const int mark, bool first_lvl);

      inline bool filterOccupied(const int i, const int x, const int y, const int mark) const {
        if(i>=(int)levels_.size())
          return false;
        if(levels_[i].data[getInd(i,x,y)].occopied==mark)
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

      /*inline bool criternion(const SubStructure::Model<Degree> &model, const SubStructure::Param<Degree> &data, const int i) const {
        f
      }*/

      /*inline bool checkModelAt(const SubStructure::Model<Degree> &model, const int i, const int x, const int y, const float thr) const {
        if(i==0)
          return model.check_tangent(levels_[i].data[getInd(i,x,y)], thr);

        return //model.check(levels_[i].data[getInd(x,y)], thr) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x,2*y)], thr) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x,2*y+1)], thr) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x+1,2*y)], thr) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)], thr);
      }*/

      inline bool checkModelAt(const SubStructure::Model<Degree> &model, const int i, const int x, const int y) const {
        if(i==0)
          return model.check_tangent(levels_[0].data[getInd(0,x,y)], 1.5f*camera_.std(levels_[0].data[getInd(0,x,y)].z_(0)/levels_[0].data[getInd(0,x,y)].model_(0,0)));

        return //model.check(levels_[i].data[getInd(x,y)], thr) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x,2*y)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x,2*y)].z_(0)/levels_[i-1].data[getInd(i-1,2*x,2*y)].model_(0,0))) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x,2*y+1)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x,2*y+1)].z_(0)/levels_[i-1].data[getInd(i-1,2*x,2*y+1)].model_(0,0))) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x+1,2*y)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x+1,2*y)].z_(0)/levels_[i-1].data[getInd(i-1,2*x+1,2*y)].model_(0,0))) &&
            model.check_tangent(levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)].z_(0)/levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)].model_(0,0)));
      }

      inline void addPoint(const int i, const int x, const int y, const int mark, S_POLYGON<Degree> &poly, const SubStructure::Model<Degree> &model, const float v=0.f) {

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

        Eigen::Vector3f p = poly.project2plane(((x<<(i+fact-1))-camera_.dx)/camera_.f,
                                               ((y<<(i+fact-1))-camera_.dy)/camera_.f,
                                               model,v);
#if 1
	bool found = false;
	const float thr = 25*camera_.std(p(2))*camera_.std(p(2));
        Eigen::Vector3f vp = poly.project2world(p.head<2>());

        static const int rel_motion_pattern[][2] = { {0,0}, {-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}
        , {-2,-2}, {-2,-1}, {-2,0}, {-2,1}, {-2,2}, {-1,-2}, {-1,2}, {0,-2}, {0,2}, {1,-2}, {1,2}, {2,-2}, {2,-1}, {2,0}, {2,1}, {2,2}
        //, {-3,-3}, {-3,-2}, {-3,-1}, {-3,0}, {-3,1}, {-3,2}, {-3,3}, {-2,-3}, {-2,3}, {-1,-3}, {-1,3}, {0,-3}, {0,3}, {1,-3}, {1,3}, {2,-3}, {2,3}, {3,-3}, {3,-2}, {3,-1}, {3,0}, {3,1}, {3,2}, {3,3}, {-4,-4}, {-4,-3}, {-4,-2}, {-4,-1}, {-4,0}, {-4,1}, {-4,2}, {-4,3}, {-4,4}, {-3,-4}, {-3,4}, {-2,-4}, {-2,4}, {-1,-4}, {-1,4}, {0,-4}, {0,4}, {1,-4}, {1,4}, {2,-4}, {2,4}, {3,-4}, {3,4}, {4,-4}, {4,-3}, {4,-2}, {4,-1}, {4,0}, {4,1}, {4,2}, {4,3}, {4,4}
        };

        const pcl::PointCloud<Point> &pc = *input_;
        for(size_t j=0; j<sizeof(rel_motion_pattern)/sizeof(rel_motion_pattern[0]); j++) {
          int xx=rel_motion_pattern[j][0];
          int yy=rel_motion_pattern[j][1];

          if(x+xx>=0 && y+yy>=0 && x+xx<(int)pc.width && y+yy<(int)pc.height) {

            if( (vp-pc(x+xx,y+yy).getVector3fMap()).squaredNorm() < thr )
            {
              found = true;
              break;
            }
          }
        }


        if(found)
        {
          poly.segments_.back().push_back(p);
#ifdef USE_BOOST_POLYGONS_
          poly.segments2d_.back().push_back(BoostPoint(x,y));
#else
          Eigen::Vector2i p2;
          p2(0) = x;
          p2(1) = y;
          poly.segments2d_.back().push_back(p2);
#endif
        }


#else
#ifndef EVALUATE

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
        for(size_t j=0; j<sizeof(rel_motion_pattern)/sizeof(rel_motion_pattern[0]); j++) {
          int xx=rel_motion_pattern[j][0];
          int yy=rel_motion_pattern[j][1];

          if(fact*x+xx>=0 && fact*y+yy>=0 && fact*x+xx<(int)pc.width && fact*y+yy<(int)pc.height) {
            Eigen::Vector3f p = poly.project2plane((((fact*x+xx)<<(i))-camera_.dx)/camera_.f,
                                                   (((fact*y+yy)<<(i))-camera_.dy)/camera_.f,
                                                   model,v);

            if( (poly.project2world(p.head<2>())-pc[getIndPC(pc.width, fact*x+xx,fact*y+yy)].getVector3fMap()).squaredNorm()<std::max(0.0005f,0.0005f*d)
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
#else
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
#endif

const float zzz=poly.project2world(p.head<2>())(2);
if(zzz>0.4f && zzz<8.f)
        if(poly.normalAt(p.head<2>())(2)>0.35f)
        {
          poly.segments_.back().push_back(p);
#ifdef USE_BOOST_POLYGONS_
          poly.segments2d_.back().push_back(BoostPoint(x,y));
#else
          Eigen::Vector2i p2;
          p2(0) = x;
          p2(1) = y;
          poly.segments2d_.back().push_back(p2);
#endif
        }
#endif
      }

      int getPos(int *ch, const int xx, const int yy, const int w, const int h);

      void outline(int *ch, const int w, const int h, std::vector<SubStructure::SXY> &out, const int i, S_POLYGON<Degree> &poly, const SubStructure::Model<Degree> &model, const int mark);

    public:
      /// constructor, setups variables
      QuadRegression();

      /// destructor
      virtual ~QuadRegression() {
        delete [] ch_;
        delete [] outline_check_;
      }

      /// sets preprocessed input cloud
      virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
      {
        input_ = cloud;
        if(levels_.size()==0)
          prepare(*cloud);
        camera_.getParams(*cloud);
      }

      virtual bool compute();

      /// get polygons
      const std::vector<Segmentation::S_POLYGON<Degree> > &getPolygons() const {return polygons_;}

      void setFilter(const float f) {filter_=f;}                /// setter for filter parameter (pixels/area)
      void setOnlyPlanes(const bool b) {only_planes_=b;}        /// setter for "only planes"-option (limits higher polynomials to planes afterwards)

      bool getFilter() const {return filter_;}                  /// getter for filter parameter (pixels/area)
      bool getOnlyPlanes() const {return only_planes_;}         /// getter for "only planes"-option (limits higher polynomials to planes afterwards)

      virtual void prepare(const pcl::PointCloud<Point> &pc);   /// setup level size
      virtual void buildTree(const pcl::PointCloud<Point> &pc); /// build quad-tree
      void calc();                                              /// segmentation on quad-tree
      void back_check_repeat();                                 /// repeat back check on model

#ifdef STOP_TIME
      // for evaluation purposes
      void getExecutionTimes(double &quadtree, double &growing, double &extraction)
      {
        quadtree = execution_time_quadtree_;
        extraction = execution_time_polyextraction_;
        growing = execution_time_growing_ - extraction;
      }
#endif
    };


    /**
     * downsamples input cloud to half widht and height
     */
    template <int Degree, typename Point, typename CameraModel>
    class QuadRegression_Downsampled : public QuadRegression<Degree, Point, CameraModel>
    {
      virtual void prepare(const pcl::PointCloud<Point> &pc);   /// setup level size
      virtual void buildTree(const pcl::PointCloud<Point> &pc); /// build quad-tree
    };

#include "impl/quad_regression_algo.hpp"

  }     //namespace QPPF
}       //namespace Segmentation


#endif /* QUAD_REGRESSION_ALGO_H_ */
