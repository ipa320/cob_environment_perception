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

#include <cob_3d_segmentation/eval.h>
#include "../sub_structures/labeling.h"
#include "../sub_structures/debug.h"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef DO_NOT_DOWNSAMPLE_
#define SHIFT   0
#else
#define SHIFT   1
#endif

#define KDTREE pcl::KdTreeFLANN

  template <typename Point, typename PointLabel, typename Parent>
  void Segmentation_QuadRegression<Point,PointLabel,Parent>::back_check_repeat() {
#ifdef BACK_CHECK_REPEAT
    for(size_t i=0; i<this->polygons_.size(); i++) {
      if(this->polygons_[i].segments_.size()<1) continue;

      //ROS_ERROR("start");

      for(size_t j=0; j<this->polygons_[i].segments2d_.size(); j++) {

        for(size_t k=0; k<this->polygons_[i].segments2d_[j].size(); k++)
        {

          Eigen::Vector2i start = this->polygons_[i].segments2d_[j][(k-1+this->polygons_[i].segments2d_[j].size())%this->polygons_[i].segments2d_[j].size()],
              end = this->polygons_[i].segments2d_[j][k], pos;

          const size_t n = std::max((size_t)1, (size_t)(sqrtf((end-start).squaredNorm())/5) );
          float prob = 0.f;

          //ROS_ERROR("n %d",n);
          //int m=0;

          for(size_t p=0; p<n; p++) {
            //pos
            pos = start + (end-start)*(p+1)/(float)(n+1);

            //ROS_ERROR("pos %d %d",pos(0),pos(1));

            if(pos(0)<0 || pos(1)<0 || pos(0)>=(int)levels_[0].w || pos(1)>=(int)levels_[0].h) continue;

            int o;
            for(int x=-12; x<=12; x+=3)
              for(int y=-12; y<=12; y+=3)
                if( (o = otherOccupied(0, pos(0)+x, pos(1)+y, this->polygons_[i].mark_)) != -1)
                  break;

            if(o<0||o==this->polygons_[i].mark_) continue;
            //ROS_ERROR("o %d",o);
            //++m;

            //get here
            Eigen::Vector2f pHere = this->polygons_[i].project2plane(((pos(0)<<(SHIFT))-kinect_params_.dx)/kinect_params_.f,
                                                               ((pos(1)<<(SHIFT))-kinect_params_.dy)/kinect_params_.f,
                                                               this->polygons_[i].model_,0.f).head(2);
            Eigen::Vector3f vHere = this->polygons_[i].project2world(pHere), nHere = this->polygons_[i].normalAt(pHere);

            //get there
            Eigen::Vector2f pThere = this->polygons_[o].project2plane(((pos(0)<<(SHIFT))-kinect_params_.dx)/kinect_params_.f,
                                                                ((pos(1)<<(SHIFT))-kinect_params_.dy)/kinect_params_.f,
                                                                this->polygons_[o].model_,0.f).head(2);
            Eigen::Vector3f vThere = this->polygons_[o].project2world(pHere), nThere = this->polygons_[o].normalAt(pHere);

            const float d = std::min(vThere(2),vHere(2));

            prob += (
                (vThere(2)-vHere(2))>0.01f*d*d && std::abs(nHere(2))>0.7f) ||
                (std::abs(vThere(2)-vHere(2))<0.1f*d*d && nHere.dot(nThere)<0.8f)
                ? 1.f:0.f;
          }

          this->polygons_[i].segments_[j][k](2) = prob/n;
          //ROS_ERROR("m %d",m);

        }
      }
    }

#endif
  }

  template <typename Point, typename PointLabel, typename Parent>
  bool Segmentation_QuadRegression<Point,PointLabel,Parent>::compute() {
    if(!this->Parent::compute())
      return false;

#ifdef BACK_CHECK_REPEAT
  back_check_repeat();
#endif
    return true;
  }

  template <typename Point, typename PointLabel, typename Parent>
  boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_QuadRegression<Point,PointLabel,Parent>::compute_labeled_pc()
  {
    typename pcl::PointCloud<PointLabel>::Ptr out(new pcl::PointCloud<PointLabel>);

    ROS_ASSERT(this->levels_.size()>1);

    out->width = this->levels_[0].w;
    out->height= this->levels_[0].h;
    out->resize(out->width*out->height);

    for(size_t x=0; x<this->levels_[0].w; x++)
    {
      for(size_t y=0; y<this->levels_[0].h; y++)
      {
        //position
        const int i=0;
        (*out)(x,y).x = this->levels_[0].data[this->getInd(i, x,y)].x();
        (*out)(x,y).y = this->levels_[0].data[this->getInd(i, x,y)].y();
        (*out)(x,y).z = this->levels_[0].data[this->getInd(i, x,y)].z();

        //color/label
        int mark = this->isOccupied(0,x,y);
        SetLabeledPoint<PointLabel>( (*out)(x,y), mark);
#ifdef SICK
        (*out)(x,y).intensity = (*input_)(x,y).intensity;
        (*out)(x,y).label = mark;
#endif
      }
    }

    return out;
  }

  template <typename Point, typename PointLabel, typename Parent>
  boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_QuadRegression<Point,PointLabel,Parent>::compute_reconstructed_pc()
  {
    typename pcl::PointCloud<PointLabel>::Ptr out(new pcl::PointCloud<PointLabel>);

    ROS_ASSERT(this->levels_.size()>0);

    out->width = this->levels_[0].w;
    out->height= this->levels_[0].h;
    out->resize(out->width*out->height);

    for(size_t x=0; x<this->levels_[0].w; x++)
    {
      for(size_t y=0; y<this->levels_[0].h; y++)
      {
        //position
        (*out)(x,y).x = this->levels_[0].data[this->getInd(0, x,y)].x();
        (*out)(x,y).y = this->levels_[0].data[this->getInd(0, x,y)].y();

        //color/label
        int mark = this->isOccupied(0,x,y);
        if(mark>=0)
          (*out)(x,y).z = this->polygons_[mark].model_.model(
              this->levels_[0].data[this->getInd(0, x,y)].x(),
              this->levels_[0].data[this->getInd(0, x,y)].y()
          );
        else
          (*out)(x,y).x = (*out)(x,y).y = (*out)(x,y).z = 0;
        SetLabeledPoint<PointLabel>( (*out)(x,y), mark);

        /*if(mark>0 && mark<(int)this->polygons_.size())
        {
          const float z = this->levels_[0].data[this->getInd(i, x,y)].z_(0)/this->levels_[0].data[this->getInd(i, x,y)].model_(0,0);

          Eigen::Vector3f p;
          p(0) =
              this->levels_[0].data[this->getInd(0, x,y)].x();
          p(1) =
              this->levels_[0].data[this->getInd(0, x,y)].y();
          p(2) = z;

          const float z_model = polygons_[mark].model_.model(
              levels_[0].data[getInd(x,y)].model_(0,1)/levels_[0].data[getInd(x,y)].model_(0,0),
              levels_[0].data[getInd(x,y)].model_(0,3)/levels_[0].data[getInd(x,y)].model_(0,0)
          );
          Point ps;
          ps.x = (*out)(x,y).x;
          ps.y = (*out)(x,y).y;
          ps.z = (*out)(x,y).z;
          kdtree.nearestKSearch(ps, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
          const float d = std::min(pointNKNSquaredDistance[0], std::min(std::abs(z - z_model), (polygons_[i].project2world(polygons_[i].nextPoint(p))-p).norm()));

          if(pcl_isfinite(d)) {
            if(d<0.1)
              (*out)(x,y).x = (*out)(x,y).y = (*out)(x,y).z = 0;
            else {
              std::cout<<"meas\n"<<p<<"\n\n";
              std::cout<<"z\n"<<z_model<<"\n";
              std::cout<<"p\n"<<polygons_[i].project2world(polygons_[i].nextPoint(p))<<"\n\n";
            }
          }

          //const float d = std::min(std::abs(z - z_model), (polygons_[i].project2world(polygons_[i].nextPoint(p))-p).norm());
          //(*out)(x,y).r = (*out)(x,y).g = (*out)(x,y).b = d/0.1f * 255;
        }*/

      }
    }

    return out;
  }

  template <typename Point, typename PointLabel, typename Parent>
  void Segmentation_QuadRegression<Point,PointLabel,Parent>::compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive)
  {
    typename pcl::PointCloud<PointLabel>::Ptr out(new pcl::PointCloud<PointLabel>);

    KDTREE<Point> kdtree;

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    kdtree.setInputCloud (this->input_);

    ROS_ASSERT(this->levels_.size()>0);

    out->width = this->levels_[0].w;
    out->height= this->levels_[0].h;
    out->resize(out->width*out->height);

    BinaryClassification bc;
    RunningStat rstat, rstat_weighted;
    points = 0;
    avg_dist = 0;

    bc.addPC(labeled_pc);

    for(size_t x=0; x<this->levels_[0].w; x++)
    {
      for(size_t y=0; y<this->levels_[0].h; y++)
      {
        //position
        const int i=0;

        //color/label
        int mark = this->isOccupied(0,x,y);

        if(mark>=0 && mark<(int)this->polygons_.size())
        {
          const float z_model = this->polygons_[mark].model_.model(
              this->levels_[0].data[this->getInd(i, x,y)].x(),
              this->levels_[0].data[this->getInd(i, x,y)].y()
          );
          const float z = this->levels_[0].data[this->getInd(i, x,y)].z_(0)/this->levels_[0].data[this->getInd(i, x,y)].model_(0,0);

          Eigen::Vector3f p;
          p(0) =
              this->levels_[0].data[this->getInd(i, x,y)].x();
          p(1) =
              this->levels_[0].data[this->getInd(i, x,y)].y();
          p(2) = z;

          Point ps;
          ps.x = p(0);
          ps.y = p(1);
          ps.z = z_model;
          if(pcl_isfinite(ps.x) && pcl_isfinite(ps.y) && pcl_isfinite(z)) {
          kdtree.nearestKSearch(ps, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
          const float d = std::min(std::sqrt(pointNKNSquaredDistance[0]), std::min(std::abs(z - z_model), (this->polygons_[i].project2world(this->polygons_[i].nextPoint(p))-p).norm()));

          if(labeled_pc && pointIdxNKNSearch.size()>0 && pointIdxNKNSearch[0]<(int)labeled_pc->size())
            bc.addMark(mark, (*labeled_pc)[pointIdxNKNSearch[0]].label);

          if(pcl_isfinite(d) && pcl_isfinite(z) && d<0.25f)
          {
            if(d<0.05f) {
              rstat.Push(d);
              rstat_weighted.Push(d, 1/z_model);
            }
            avg_dist += z;
          }}
        }

        if(this->levels_[0].data[this->getInd(i, x,y)].z_(0)/this->levels_[0].data[this->getInd(i, x,y)].model_(0,0)>0 && pcl_isfinite(this->levels_[0].data[this->getInd(i, x,y)].z_(0)/this->levels_[0].data[this->getInd(i, x,y)].model_(0,0)))
          points++;

      }
    }

    bc.finish().get_rate(true_positive, false_positive);

    //points = this->levels_[0].w*this->levels_[0].h;
    used = rstat.NumDataValues();
    mem = 0;
    for(size_t i=0; i<this->polygons_.size(); i++)
      mem+=4*SubStructure::Param<Parent::DEGREE>::NUM + this->polygons_[i].segments_.size()*2*4;

    mean = rstat.Mean();
    var = rstat.Variance();
    mean_weighted = rstat_weighted.Mean();
    var_weighted = rstat_weighted.Variance();
    avg_dist /= points;
  }

  template <typename Point, typename PointLabel, typename Parent>
  Segmentation_QuadRegression<Point,PointLabel,Parent>::operator cob_3d_mapping_msgs::ShapeArray() const {
    cob_3d_mapping_msgs::ShapeArray sa;

    cob_3d_mapping_msgs::Shape s;
    if(this->getOnlyPlanes())
      s.type = cob_3d_mapping_msgs::Shape::POLYGON;
    else
      s.type = cob_3d_mapping_msgs::Shape::CURVED;

    sa.header.frame_id = s.header.frame_id = "/test";

    for(size_t i=0; i<this->polygons_.size(); i++) {
      if(this->polygons_[i].segments_.size()<1) continue;

      Eigen::Vector3f mi, ma;

      s.params.clear();

      if(this->getOnlyPlanes()) {
        /*ROS_ASSERT( std::abs(this->polygons_[i].model_.p(2))<0.001f &&
                    std::abs(this->polygons_[i].model_.p(4))<0.001f &&
                    std::abs(this->polygons_[i].model_.p(5))<0.001f );*/

        Eigen::Vector3f n = this->polygons_[i].model_.getNormal(0,0), v;
        v.fill(0);
        v(2) = this->polygons_[i].model_.p(0);
        for(int i=0; i<3; i++) s.params.push_back(n(i));
        s.params.push_back(n.dot(v));
      }
      else {
        for(int k=0; k<SubStructure::Param<Parent::DEGREE>::NUM; k++)
          s.params.push_back(this->polygons_[i].model_.p(k));
      }

      s.pose.position.x = this->polygons_[i].model_.x();
      s.pose.position.y = this->polygons_[i].model_.y();
      s.pose.position.z = this->polygons_[i].model_.z(); //perhaps from model?
      
      Eigen::Quaternionf orientation = this->polygons_[i].get_orientation();
      s.pose.orientation.x = orientation.x();
      s.pose.orientation.y = orientation.y();
      s.pose.orientation.z = orientation.z();
      s.pose.orientation.w = orientation.w();
      
      const Eigen::Affine3f T = (
				Eigen::Translation3f(s.pose.position.x,s.pose.position.y,s.pose.position.z)*
				orientation
			).inverse();

      s.weight = this->polygons_[i].weight_;

      s.color.r=this->polygons_[i].color_[0];
      s.color.g=this->polygons_[i].color_[1];
      s.color.b=this->polygons_[i].color_[2];
      s.color.a=1.f;

      s.id = i;
      //s.color.a=std::min(1000.f,this->polygons_[i].weight_)/1000.f;

      s.points.clear();
      float backs=0;
      for(size_t j=0; j<this->polygons_[i].segments_.size(); j++) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::PointXYZ pt;

        for(size_t k=0; k<this->polygons_[i].segments_[j].size(); k++) {
			pt.x=this->polygons_[i].segments_[j][k](0);
			pt.y=this->polygons_[i].segments_[j][k](1);
			
			//compatible to code of goa (e.g. visualization)
			if(this->getOnlyPlanes()) {
			  const Eigen::Vector3f p = T*this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2) );
			  
			  pt.x = p(0);
			  pt.y = p(1);
			  pt.z = p(2);
			  
			  if(pcl_isfinite(pt.x) && pcl_isfinite(pt.y))
				pc.push_back(pt);
			}
			else {
			  pt.z=this->polygons_[i].segments_[j][k](2);
			  if(j==0) {
				backs+=this->polygons_[i].segments_[j][k](2);
				if(k==0)
				  mi = ma = this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2) );
				else
				{
				  Eigen::Vector3f t = this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2) );
				  mi(0) = std::min(t(0),mi(0));
				  mi(1) = std::min(t(1),mi(1));
				  mi(2) = std::min(t(2),mi(2));
				  ma(0) = std::max(t(0),ma(0));
				  ma(1) = std::max(t(1),ma(1));
				  ma(2) = std::max(t(2),ma(2));
				}
			  }
			  if(pcl_isfinite(pt.x) && pcl_isfinite(pt.y)) {
				pc.push_back(pt);
			  }
		  }
		  
        }

        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(pc, pc2);
        sensor_msgs::PointCloud2 pc_msg;
        pcl_conversions::fromPCL(pc2, pc_msg);   
        //pcl::toROSMsg(pc,pc_msg);

        s.points.push_back(pc_msg);
        s.holes.push_back(j>0);
      }

      //ROS_INFO("density %f",(mi-ma).squaredNorm()/(this->polygons_[i].weight_*this->polygons_[i].weight_));
      //if( (mi-ma).squaredNorm()/(this->polygons_[i].weight_*this->polygons_[i].weight_)<0.00002f)
      sa.shapes.push_back(s);
    }

    return sa;
  }


  template <typename Point, typename PointLabel, typename Parent>
  Segmentation_QuadRegression<Point,PointLabel,Parent>::operator visualization_msgs::Marker() const {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.002;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;

    for(size_t i=0; i<this->polygons_.size(); i++) {


//      std::cerr<<"OFF:\n"<<this->polygons_[i].param_.col(0)<<"\n";
//      std::cerr<<"PLANE:\n"<<this->polygons_[i].proj2plane_<<"\n";
//      std::cerr<<"P1:\n"<<this->polygons_[i].param_.col(1)<<"\n";
//      std::cerr<<"P2:\n"<<this->polygons_[i].param_.col(2)<<"\n";

      if(this->polygons_[i].segments_.size()<1) continue;

      for(size_t j=0; j<this->polygons_[i].segments_.size(); j++) {

        for(size_t k=0; k<this->polygons_[i].segments_[j].size(); k++)
        {
          std_msgs::ColorRGBA c;
          geometry_msgs::Point p;
          Eigen::Vector3f v1,v2;

          c.r=this->polygons_[i].segments_[j][(k+1)%this->polygons_[i].segments_[j].size()](2);
          c.g=0;
          c.b=1-c.r;
          c.a=1;

          v1 = this->polygons_[i].project2world(this->polygons_[i].segments_[j][k].head(2));
          v2 = this->polygons_[i].project2world(this->polygons_[i].segments_[j][(k+1)%this->polygons_[i].segments_[j].size()].head(2));


//          std::cerr<<"s\n"<<this->polygons_[i].segments_[j][k].head<2>()<<"\n";
//          std::cerr<<"P\n"<<v1<<"\n";

          if(!pcl_isfinite(v1.sum()) || !pcl_isfinite(v2.sum()))
            continue;

          //          std::cout<<"O\n"<<v1<<"\n";

          p.x = v1(0);
          p.y = v1(1);
          p.z = v1(2);
          m.points.push_back(p);
          m.colors.push_back(c);

          p.x = v2(0);
          p.y = v2(1);
          p.z = v2(2);
          m.points.push_back(p);
          m.colors.push_back(c);
        }
      }
    }

    return m;
  }

  template <typename Point, typename PointLabel, typename Parent>
  bool Segmentation_QuadRegression<Point,PointLabel,Parent>::extractImages() {
#ifdef USE_COLOR
    const pcl::PointCloud<Point> &pc = (*this->input_);

    for(size_t i=0; i<this->polygons_.size(); i++) {
      this->polygons_[i].img_.reset(new sensor_msgs::Image);

      if(this->polygons_[i].segments2d_.size()<1 || this->polygons_[i].segments2d_[0].size()<3) continue;

      //2d rect
      int mix=this->input_->width, miy=this->input_->height, max=-1, may=-1;
      for(size_t j=0; j<this->polygons_[i].segments2d_[0].size(); j++) { //outer hull
        mix = std::min(mix, this->polygons_[i].segments2d_[0][j](0));
        max = std::max(max, this->polygons_[i].segments2d_[0][j](0));
        miy = std::min(miy, this->polygons_[i].segments2d_[0][j](1));
        may = std::max(may, this->polygons_[i].segments2d_[0][j](1));
      }
#ifndef DO_NOT_DOWNSAMPLE_
      mix*=2;
      max*=2;
      miy*=2;
      may*=2;
#endif

      int w=max-mix, h=may-miy;
      max-=w/4;
      mix+=w/4;
      may-=h/4;
      miy+=h/4;

      this->polygons_[i].img_->width  = max-mix+1;
      this->polygons_[i].img_->height = may-miy+1;
      this->polygons_[i].img_->encoding = "rgb8";
      this->polygons_[i].img_->step = this->polygons_[i].img_->width*3;
      this->polygons_[i].img_->is_bigendian = false;
      this->polygons_[i].img_->data.resize( this->polygons_[i].img_->step*this->polygons_[i].img_->height );

      this->polygons_[i].color_[0] = this->polygons_[i].color_[1] = this->polygons_[i].color_[2] = 0.f;
      for(int x=mix; x<=max; x++) {
        for(int y=miy; y<=may; y++) {
          this->polygons_[i].img_->data[(y-miy)*this->polygons_[i].img_->step + 3*(x-mix) + 0] = pc(x,y).r;
          this->polygons_[i].img_->data[(y-miy)*this->polygons_[i].img_->step + 3*(x-mix) + 1] = pc(x,y).g;
          this->polygons_[i].img_->data[(y-miy)*this->polygons_[i].img_->step + 3*(x-mix) + 2] = pc(x,y).b;

          this->polygons_[i].color_[0] += pc(x,y).r;
          this->polygons_[i].color_[1] += pc(x,y).g;
          this->polygons_[i].color_[2] += pc(x,y).b;
        }
      }

      this->polygons_[i].color_[0] /= 255.f*this->polygons_[i].img_->width*this->polygons_[i].img_->height;
      this->polygons_[i].color_[1] /= 255.f*this->polygons_[i].img_->width*this->polygons_[i].img_->height;
      this->polygons_[i].color_[2] /= 255.f*this->polygons_[i].img_->width*this->polygons_[i].img_->height;
    }
  return true;
#else
  return false;
#endif
  }


  template <typename Point, typename PointLabel, typename Parent>
  std::istream &Segmentation_QuadRegression<Point,PointLabel,Parent>::serialize(std::istream &is) {
	this->polygons_.clear();

	int degree=0; is.read((char*)&degree, sizeof(degree));
	assert(degree==Parent::DEGREE);

	size_t num_polygons=0;
	is.read((char*)&num_polygons, sizeof(num_polygons));
	this->polygons_.resize(num_polygons);
	for(size_t i=0; i<num_polygons; i++) {
		for(int j=0; j<this->polygons_[i].model_.param.NUM; j++) {
			float f=0;
			is.read((char*)&f, sizeof(f));
			this->polygons_[i].model_.p[j] = f;
		}
		size_t num_hulls=0;
		is.read((char*)&num_hulls, sizeof(num_hulls));
		this->polygons_[i].segments_.resize(num_hulls);
		for(size_t j=0; j<num_hulls; j++) {
			size_t num_pts=0;
			is.read((char*)&num_pts, sizeof(num_pts));
			this->polygons_[i].segments_[j].resize(num_pts);
			for(size_t k=0; k<num_pts; k++) {
				for(int l=0; l<2; l++) {
					float f=0;
					is.read((char*)&f, sizeof(f));
					this->polygons_[i].segments_[j][k](l) = f;
				}
			}
		}
	}

	return is;
   }

  template <typename Point, typename PointLabel, typename Parent>
  std::ostream &Segmentation_QuadRegression<Point,PointLabel,Parent>::serialize(std::ostream &os) const {
	size_t num_polygons=this->polygons_.size();

	int degree=Parent::DEGREE; os.write((const char*)&degree, sizeof(degree));

	os.write((const char*)&num_polygons, sizeof(num_polygons));
	for(size_t i=0; i<num_polygons; i++) {
		for(int j=0; j<this->polygons_[i].model_.param.NUM; j++) {
			float f=this->polygons_[i].model_.p[j];
			os.write((const char*)&f, sizeof(f));
		}
		size_t num_hulls=this->polygons_[i].segments_.size();
		os.write((const char*)&num_hulls, sizeof(num_hulls));
		for(size_t j=0; j<num_hulls; j++) {
			size_t num_pts=this->polygons_[i].segments_[j].size();
			os.write((const char*)&num_pts, sizeof(num_pts));
			for(size_t k=0; k<num_pts; k++) {
				for(int l=0; l<2; l++) {
					float f=this->polygons_[i].segments_[j][k](l);
					os.write((const char*)&f, sizeof(f));
				}
			}
		}
	}

	return os;
   }

  template <typename Point, typename PointLabel, typename Parent>
  Segmentation_QuadRegression<Point,PointLabel,Parent>::operator cob_3d_mapping_msgs::PlaneScene() const {
    cob_3d_mapping_msgs::PlaneScene ps;
    const float pixel_var = 2;

    for(size_t i=0; i<this->polygons_.size(); i++) {
      if(this->polygons_[i].segments_.size()<1) continue;
      
      cob_3d_mapping_msgs::Plane plane;

      plane.pose.position.x = this->polygons_[i].model_.x();
      plane.pose.position.y = this->polygons_[i].model_.y();
      plane.pose.position.z = this->polygons_[i].model_.z(); //perhaps from model?
      
      Eigen::Quaternionf orientation = this->polygons_[i].get_orientation();
      plane.pose.orientation.x = orientation.x();
      plane.pose.orientation.y = orientation.y();
      plane.pose.orientation.z = orientation.z();
      plane.pose.orientation.w = orientation.w();
      
      const Eigen::Affine3f T = (
				Eigen::Translation3f(plane.pose.position.x,plane.pose.position.y,plane.pose.position.z)*
				orientation
			).inverse();

      plane.weight = this->polygons_[i].weight_;

      plane.color.r=this->polygons_[i].color_[0];
      plane.color.g=this->polygons_[i].color_[1];
      plane.color.b=this->polygons_[i].color_[2];
      plane.color.a=1.f;
      
      for(size_t j=0; j<this->polygons_[i].segments_.size(); j++) {
        cob_3d_mapping_msgs::Polygon poly;
        
        for(size_t k=0; k<this->polygons_[i].segments_[j].size(); k++) {
          const Eigen::Vector3f o = this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2) );
          const Eigen::Vector3f p = T*o;
          cob_3d_mapping_msgs::Point2D pt;
			  
          pt.x = p(0);
          pt.y = p(1);
          
          //TODO:
          pt.tex_x = this->polygons_[i].segments2d_[j][k](0)/(float)this->levels_[0].w;
          pt.tex_y = this->polygons_[i].segments2d_[j][k](1)/(float)this->levels_[0].h;
          
          pt.var = this->camera_.std(o(2));
          
          Eigen::Vector3f t;
          
          pt.var = std::max(pt.var, (this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2)+Eigen::Vector2f(pixel_var*o(2)/this->camera_.f, 0.f) )-o).norm());
          pt.var = std::max(pt.var, (this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2)+Eigen::Vector2f(-pixel_var*o(2)/this->camera_.f, 0.f) )-o).norm());
          
          pt.var = std::max(pt.var, (this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2)+Eigen::Vector2f(0.f, pixel_var*o(2)/this->camera_.f) )-o).norm());
          pt.var = std::max(pt.var, (this->polygons_[i].project2world( this->polygons_[i].segments_[j][k].head(2)+Eigen::Vector2f(0.f, -pixel_var*o(2)/this->camera_.f) )-o).norm());
          
          poly.points.push_back(pt);
        }
        plane.polygons.push_back(poly);
      }
      
      ps.planes.push_back(plane);
    }
    
    return ps;
  }
  
