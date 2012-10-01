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
 * segments.h
 *
 *  Created on: Nov 28, 2011
 *      Author: goa-jh
 */

#ifndef SEGMENTS_H_
#define SEGMENTS_H_

#include <pcl/filters/extract_indices.h>
#ifdef PCL_VERSION_COMPARE
  #include <pcl/registration/impl/correspondence_types.hpp>
  #include <pcl/PointIndices.h>
#endif
#ifdef VISUALIZE_SEGMENTS_
#include <pcl/visualization/cloud_viewer.h>
#endif

struct EIGEN_ALIGN16 PCAPoint
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

  float data_c[9];

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename Point>
class Keypoints_Segments : public RegKeypointCorrespondence<Point, PCAPoint>
{
  float radius_;
  float thr_;
  float dist_threshold_;

  Point keypoint2point(const PCAPoint &kp) {
    Point p;
    p.x=kp.x;
    p.y=kp.y;
    p.z=kp.z;
    return p;
  }

public:
  Keypoints_Segments():
    radius_(0.05), thr_(0.1), dist_threshold_(0.1)
  {}

  void setRadius(float v) {radius_ = v;}
  void setThreshold(float v) {thr_ = v;}
  void setDisThreshold(float v) {dist_threshold_ = v;}

  void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<Point> &narf_descriptors, pcl::PointCloud<PCAPoint> &mids);
#ifdef VISUALIZE_SEGMENTS_
  void
  show (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
  {
    //... populate cloud
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    viewer.showCloud (cloud2);
    while (!viewer.wasStopped ())
    {
    }
  }
#endif

  virtual bool compute(const pcl::PointCloud<Point> &src, const pcl::PointCloud<Point> &tgt) {
    pcl::PointCloud<Point> dummy, dummy2;
    extractFeatures(src, dummy, this->keypoints_src_);
    extractFeatures(tgt, dummy2, this->keypoints_tgt_);

    ROS_INFO("found %d keypoints in src", this->keypoints_src_.size());
    ROS_INFO("found %d keypoints in tgt", this->keypoints_tgt_.size());

#ifdef VISUALIZE_SEGMENTS_
    show(dummy.makeShared(), dummy2.makeShared());
#endif
  }

  inline Eigen::Vector3f getEigenVector(const PCAPoint &kp, const int r) {
    Eigen::Vector3f ret;
    ret(0) = kp.data_c[3*r+0];
    ret(1) = kp.data_c[3*r+1];
    ret(2) = kp.data_c[3*r+2];
    return ret;
  }
  #ifdef PCL_VERSION_COMPARE
    virtual void getCorrespondences(pcl::Correspondences &correspondences) {

      pcl::Correspondence corr;
  #else
    virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences) {

      pcl::registration::Correspondence corr;
  #endif
    correspondences.clear();

    const float thr = 0.001;
    const float max_dis = 0.5*0.5;

    for(int i=0; i<this->keypoints_src_.size(); i++) {
      int mi=0;
      float f=111111;

      Eigen::Vector3f ev1=getEigenVector(this->keypoints_src_[i],0);
      Eigen::Vector3f ev2=getEigenVector(this->keypoints_src_[i],1);
      Eigen::Vector3f ev3=getEigenVector(this->keypoints_src_[i],2);

      for(int j=0; j<this->keypoints_tgt_.size(); j++) {
        Eigen::Vector3f ev1t=getEigenVector(this->keypoints_tgt_[j],0);
        Eigen::Vector3f ev2t=getEigenVector(this->keypoints_tgt_[j],1);
        Eigen::Vector3f ev3t=getEigenVector(this->keypoints_tgt_[j],2);
        Eigen::Vector3f v=this->keypoints_src_[i].getVector3fMap()-this->keypoints_tgt_[j].getVector3fMap();
        float dis = (std::abs(acosf(ev3.dot(ev3t)/(ev3.norm()*ev3t.norm())))+0.1)
            *(std::abs(ev1.squaredNorm()-ev1t.squaredNorm())+std::abs(ev2.squaredNorm()-ev2t.squaredNorm()))*v.norm();//(ev1-ev1t).squaredNorm()*(ev2-ev2t).squaredNorm()*v.squaredNorm();

        if(dis<f && v.squaredNorm()<max_dis) {
          f=dis;
          mi=j;
        }

      }

      if(f>thr)
        continue;
	  #ifdef PCL_VERSION_COMPARE
        corr.index_query = i;
        corr.index_match = mi;
      #else
        corr.indexQuery = i;
        corr.indexMatch = mi;
      #endif
      corr.distance = f;
      correspondences.push_back(corr);

      Eigen::Vector3f v=this->keypoints_src_[i].getVector3fMap()-this->keypoints_tgt_[mi].getVector3fMap();
      ROS_INFO("corr %d -> %d with %f %f", i, mi, f, v.norm());
    }
  }


  Point getPointForKeypointSrc(const int ind) {return keypoint2point(this->keypoints_src_[ind]);}
  Point getPointForKeypointTgt(const int ind) {return keypoint2point(this->keypoints_tgt_[ind]);}
};

#include "impl/segments.hpp"

#endif /* SEGMENTS_H_ */
