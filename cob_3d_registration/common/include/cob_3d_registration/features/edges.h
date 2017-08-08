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
 * edges.h
 *
 *  Created on: Nov 18, 2011
 *      Author: goa-jh
 */

#ifndef EDGES_H_
#define EDGES_H_


#include <pcl/visualization/cloud_viewer.h>

#include "pcl/range_image/range_image.h"
#include "pcl/features/range_image_border_extractor.h"
#include "pcl/keypoints/narf_keypoint.h"
#include "pcl/features/narf_descriptor.h"
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <cob_3d_features/fast_edge_estimation_3d_omp.h>
#include <cob_3d_mapping_common/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/integral_image_normal.h>

namespace cob_3d_registration {

template<typename Point>
class Feature_Edges : public FeatureContainerInterface_Euclidean<Point>
{
  pcl::PointCloud<Point> feature_in_, feature_out_;

  float radius_;
  float thr_;
  float dist_threshold_;

public:
  Feature_Edges():
    radius_(0.05), thr_(0.5), dist_threshold_(0.1)
  {}

  void setRadius(float v) {radius_ = v;}
  void setThreshold(float v) {thr_ = v;}
  void setDisThreshold(float v) {dist_threshold_ = v;}

  void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<Point> &narf_descriptors);

  const pcl::PointCloud<Point> &getFilteredInputCloud() {return this->org_in_;}
  const pcl::PointCloud<Point> &getFilteredOutputCloud() {return this->org_out_;}

  virtual bool hidden_build() {

    ROS_INFO("calc edges for source %d", (int)this->org_in_.size());
    {
      /*extractFeatures(this->org_in_, feature_in_);
      this->org_in_.points.clear();
      for(int i=0; i<feature_in_.size(); i++) {
        Point p;
        memset(&p,0,sizeof(Point));
        p.x=feature_in_.points[i].x;
        p.y=feature_in_.points[i].y;
        p.z=feature_in_.points[i].z;
        this->org_in_.points.push_back(p);
      }
      this->org_in_.height=1;
      this->org_in_.width=this->org_in_.size();*/
      feature_in_=this->org_in_;
    }

    ROS_INFO("calc edges for target %d", (int)this->org_out_.size());
    {
      extractFeatures(this->org_out_, feature_out_);
      this->org_out_.points.clear();
      for(int i=0; i<(int)feature_out_.size(); i++) {
        Point p;
        memset(&p,0,sizeof(Point));
        p.x=feature_out_.points[i].x;
        p.y=feature_out_.points[i].y;
        p.z=feature_out_.points[i].z;
        this->org_out_.points.push_back(p);
      }
      this->org_out_.height=1;
      this->org_out_.width=this->org_out_.size();
    }

    //need to rebuild tree
    //ROS_INFO("build tree %d %d", this->org_in_.size(), this->org_out_.size());
#ifdef PCL_VERSION_COMPARE
    this->tree_.reset (new pcl::search::KdTree<Point>);
#else
	this->tree_.reset (new pcl::KdTreeFLANN<Point>);
#endif
    if(this->org_in_.size()>0) this->tree_->setInputCloud(this->org_in_.makeShared());
    //ROS_INFO("build tree done");

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    return this->org_out_.points[index].getVector3fMap();
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    return this->org_in_.points[index].getVector3fMap();
  }
};

#include "impl/edges.hpp"
}

#endif /* EDGES_H_ */

