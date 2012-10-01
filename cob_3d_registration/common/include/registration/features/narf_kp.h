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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
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
 * narf_kp.h
 *
 *  Created on: Nov 29, 2011
 *      Author: goa-jh
 */

#ifndef NARF_KP_H_
#define NARF_KP_H_

#include <ros/console.h>
#ifdef PCL_VERSION_COMPARE
 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>
 #include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_traits.h>




struct NarfKPoint
 {
   PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
   pcl::FPFHSignature33 fpfh;
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
 } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

 POINT_CLOUD_REGISTER_POINT_STRUCT (NarfKPoint,           // here we assume a XYZ + "test" (as fields)
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float[33], fpfh,fpfh)

 )
#else
struct EIGEN_ALIGN16 NarfKPoint
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

  pcl::FPFHSignature33 fpfh;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif

template<typename Point>
class Keypoints_Narf : public RegKeypointCorrespondence<Point, NarfKPoint>
{
  float radius_;
  float thr_;
  float dist_threshold_;

  Point keypoint2point(const NarfKPoint &kp) {
    Point p;
    p.x=kp.x;
    p.y=kp.y;
    p.z=kp.z;
    return p;
  }

  void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<NarfKPoint> &narf_descriptors);

public:
  Keypoints_Narf():
    radius_(0.05), thr_(0.1), dist_threshold_(0.1)
  {}

  void setRadius(float v) {radius_ = v;}
  void setThreshold(float v) {thr_ = v;}
  void setDisThreshold(float v) {dist_threshold_ = v;}

  virtual bool compute(const pcl::PointCloud<Point> &src, const pcl::PointCloud<Point> &tgt) {
    ROS_INFO("calc narf for source %d", src.size());
    {
      extractFeatures(src, this->keypoints_src_);
    }

    ROS_INFO("calc narf for target %d", tgt.size());
    {
      extractFeatures(tgt, this->keypoints_tgt_);
    }

    ROS_INFO("%d %d", this->keypoints_src_.size(), this->keypoints_tgt_.size());

    return this->keypoints_src_.size()>0 && this->keypoints_tgt_.size()>0;
  }
  #ifdef PCL_VERSION_COMPARE
    virtual void getCorrespondences(pcl::Correspondences &correspondences) {
  #else
    virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences) {
  #endif
    pcl::PointCloud<pcl::FPFHSignature33> tsrc, ttgt;

    if(this->keypoints_src_.size()<1||this->keypoints_tgt_.size()<1) {
      ROS_ERROR("cannot compute 0 correspondences!");
      return;
    }

    for(int i=0; i<this->keypoints_src_.size(); i++)
      tsrc.points.push_back(this->keypoints_src_.points[i].fpfh);
    for(int i=0; i<this->keypoints_tgt_.size(); i++)
      ttgt.points.push_back(this->keypoints_tgt_.points[i].fpfh);
    tsrc.height=ttgt.height=1;
    ttgt.width=ttgt.size();
    tsrc.width=tsrc.size();

    {
      IndicesPtr indices_;
      indices_.reset (new std::vector<int>);
      indices_->resize (tsrc.points.size ());
      for(int i=0; i<tsrc.size(); i++)
        (*indices_)[i]=i;

      // setup tree for reciprocal search
	  #ifdef PCL_VERSION_COMPARE
	    pcl::search::KdTree<pcl::FPFHSignature33> tree_reciprocal;
        pcl::search::KdTree<pcl::FPFHSignature33> tree_;
      #else
        pcl::KdTreeFLANN<pcl::FPFHSignature33> tree_reciprocal;
        pcl::KdTreeFLANN<pcl::FPFHSignature33> tree_;
      #endif
      tree_reciprocal.setInputCloud(tsrc.makeShared(), indices_);
      tree_.setInputCloud(ttgt.makeShared());

      correspondences.resize(indices_->size());
      std::vector<int> index(1);
      std::vector<float> distance(1);
      std::vector<int> index_reciprocal(1);
      std::vector<float> distance_reciprocal(1);
	  #ifdef PCL_VERSION_COMPARE
	 	pcl::Correspondence corr;
	  #else
        pcl::registration::Correspondence corr;
	  #endif
      unsigned int nr_valid_correspondences = 0;

      for (unsigned int i = 0; i < indices_->size(); ++i)
      {
        tree_.nearestKSearch(tsrc.points[(*indices_)[i]], 1, index, distance);
        tree_reciprocal.nearestKSearch(ttgt.points[index[0]], 1, index_reciprocal, distance_reciprocal);

        if ( (*indices_)[i] == index_reciprocal[0] )
        {
		  #ifdef PCL_VERSION_COMPARE
			corr.index_query = (*indices_)[i];
            corr.index_match = index[0];
		  #else
            corr.indexQuery = (*indices_)[i];
            corr.indexMatch = index[0];
		  #endif
          corr.distance = (this->keypoints_tgt_[index[0]].getVector3fMap()-this->keypoints_src_[(*indices_)[i]].getVector3fMap()).squaredNorm();
          correspondences[nr_valid_correspondences] = corr;
          ++nr_valid_correspondences;
        }
      }
      correspondences.resize(nr_valid_correspondences);
    }

    ROS_INFO("cor found %d",correspondences.size());
  }

  Point getPointForKeypointSrc(const int ind) {return keypoint2point(this->keypoints_src_[ind]);}
  Point getPointForKeypointTgt(const int ind) {return keypoint2point(this->keypoints_tgt_[ind]);}
};

#include "impl/narf_kp.hpp"

#endif /* NARF_KP_H_ */
