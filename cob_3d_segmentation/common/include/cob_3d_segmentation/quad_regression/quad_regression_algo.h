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
//#define STOP_TIME
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
      friend std::ostream &operator<<(std::ostream &os, const CameraModel_Kinect<APoint> &cmk);
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

      /// calculate parameters
      void getParams(const pcl::PointCloud<Point> &pc);

      inline static float std(const float dist) {return 0.00015f+0.00015f*dist;}   //after "Accuracy analysis of kinect depth data"

      /** DEBUG **/
      template<typename APoint>
      friend std::ostream &operator<<(std::ostream &os, const CameraModel_Kinect<APoint> &cmk);
    };

    /**
     * camera model for Ensenso
     *
     *  - extracts parameters from pc
     *  - states standard deviation for a given distance
     */
    template<typename Point>
    class CameraModel_Ensenso : public CameraModel_Kinect<Point> {
    public:
      CameraModel_Ensenso(){}

      inline static float std(const float dist) {return 0.009666969f*0.5f*dist*dist;}
    };

    template<typename Point>
    std::ostream &operator<<(std::ostream &os, const CameraModel_Kinect<Point> &cmk) {
      os<<"Ensenso Camera\n";
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
      CameraModel camera_;              /// camera model

#ifdef STOP_TIME
      // evaluation variables
      double execution_time_quadtree_, execution_time_growing_, execution_time_polyextraction_;
#endif

      enum {DEGREE=Degree};             // constant for degree (1=linear)

    private:
      //-----------CONSTANTS-------------
      const unsigned int NUM_LOD;
      const unsigned int MIN_LOD;       /// minimum of nodes for a segment
      const unsigned int FINAL_LOD;     /// lowest level = 0
      const unsigned int GO_DOWN_TO_LVL;/// search down to ...

      //------------MEMBERS---------------

      int *ch_; /// mark-array

      bool *outline_check_;             /// needed for outline, no need to reallocate every time
      size_t outline_check_size_;       /// remember size for var. above
      float filter_;                    /// ratio points per area
      bool only_planes_;                /// filter option

      void grow(SubStructure::Model<Degree> &model, const int i, const int x, const int y);
      void grow(SubStructure::VISITED_LIST<SubStructure::SVALUE<Eigen::Vector3f> > &list, SubStructure::Model<Degree> &model, const int i, const int mark, bool first_lvl);

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
        if(levels_[i].data[getInd(i,x,y)].occopied>=0 && levels_[i].data[getInd(i,x,y)].occopied!=mark)
          return levels_[i].data[getInd(i,x,y)].occopied;
        return otherOccupied(i+1,x/2,y/2,mark);
      }

    inline bool replaceOccupied(const int i, const int x, const int y, const int mark) const {
      if(i>=(int)levels_.size() || x<0 || y<0 || x>=(int)levels_[i].w || y>=(int)levels_[i].h)
        return false;
      if(levels_[i].data[getInd(i,x,y)].occopied!=mark)
        return false;
      
      levels_[i-1].data[getInd(i-1, 2*x,  2*y)].occopied   = mark;
      levels_[i-1].data[getInd(i-1, 2*x+1,2*y)].occopied   = mark;
      levels_[i-1].data[getInd(i-1, 2*x+1,2*y+1)].occopied = mark;
      levels_[i-1].data[getInd(i-1, 2*x,  2*y+1)].occopied = mark;
      
      return true;
    }
    
    inline bool checkModelAt(const SubStructure::Model<Degree> &model, const int i, const int x, const int y) const {
      if(i==0)
        return model.check_tangent(levels_[0].data[getInd(0,x,y)], 1.5f*camera_.std(levels_[0].data[getInd(0,x,y)].z_(0)/levels_[0].data[getInd(0,x,y)].model_(0,0)));

      return //model.check(levels_[i].data[getInd(x,y)], thr) &&
        model.check_tangent(levels_[i-1].data[getInd(i-1,2*x,2*y)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x,2*y)].z_(0)/levels_[i-1].data[getInd(i-1,2*x,2*y)].model_(0,0))) &&
        model.check_tangent(levels_[i-1].data[getInd(i-1,2*x,2*y+1)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x,2*y+1)].z_(0)/levels_[i-1].data[getInd(i-1,2*x,2*y+1)].model_(0,0))) &&
        model.check_tangent(levels_[i-1].data[getInd(i-1,2*x+1,2*y)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x+1,2*y)].z_(0)/levels_[i-1].data[getInd(i-1,2*x+1,2*y)].model_(0,0))) &&
        model.check_tangent(levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)], 1.5f*camera_.std(levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)].z_(0)/levels_[i-1].data[getInd(i-1,2*x+1,2*y+1)].model_(0,0)));
    }

    inline bool addPoint(const int i, const int x, const int y, const int mark, S_POLYGON<Degree> &poly, const SubStructure::Model<Degree> &model, const Eigen::Vector3f &p) {
#ifdef DO_NOT_DOWNSAMPLE_
      const static int fact=1;
#else
      const static int fact=2;
#endif

      Eigen::Vector3f vp = poly.project2plane(((x<<(i+fact-1))-camera_.dx)/camera_.f,
                                               ((y<<(i+fact-1))-camera_.dy)/camera_.f,
                                               model,0.f);
      poly.segments_.back().push_back(vp);
#ifdef USE_BOOST_POLYGONS_
      poly.segments2d_.back().push_back(BoostPoint(x,y));
#else
      Eigen::Vector2i p2;
      p2(0) = x;
      p2(1) = y;
      poly.segments2d_.back().push_back(p2);
#endif

      const float thr = 25*camera_.std(vp(2))*camera_.std(vp(2)) + 0.01f;
      return (p-vp).squaredNorm()<thr;
    }

    int getPos(int *ch, const int xx, const int yy, const int w, const int h);

    void outline(int *ch, const int w, const int h, std::vector<SubStructure::SXY<Eigen::Vector3f> > &out, const int i, S_POLYGON<Degree> &poly, const SubStructure::Model<Degree> &model, const int mark);
    
    void simplify();
    void simplify(const float scale, std::vector<Eigen::Vector3f> &seg, std::vector<Eigen::Vector2i> &seg2d);      

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
