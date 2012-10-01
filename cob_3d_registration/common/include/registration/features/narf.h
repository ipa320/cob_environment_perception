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
 * narf.h
 *
 *  Created on: Nov 15, 2011
 *      Author: goa-jh
 */

#ifndef NARF_H_
#define NARF_H_

#include "pcl/range_image/range_image.h"
#include "pcl/features/range_image_border_extractor.h"
#include "pcl/keypoints/narf_keypoint.h"
#include "pcl/features/narf_descriptor.h"
#include <pcl/filters/extract_indices.h>


template<typename Point>
class Feature_NARF : public FeatureContainerInterface_Euclidean<Point>
{
  pcl::PointCloud<pcl::Narf36> feature_in_, feature_out_;

public:
  static void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<pcl::Narf36> &narf_descriptors);

  const pcl::PointCloud<Point> &getFilteredInputCloud() {return this->org_in_;}
  const pcl::PointCloud<Point> &getFilteredOutputCloud() {return this->org_out_;}

  virtual bool hidden_build() {

    ROS_INFO("calc narf for source %d", this->org_in_.size());
    {
      extractFeatures(this->org_in_, feature_in_);
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
      this->org_in_.width=this->org_in_.size();
    }

    ROS_INFO("calc narf for target %d", this->org_out_.size());
    {
      extractFeatures(this->org_out_, feature_out_);
      this->org_out_.points.clear();
      for(int i=0; i<feature_out_.size(); i++) {
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
    ROS_INFO("build tree %d %d", this->org_in_.size(), this->org_out_.size());
    this->tree_.reset (new pcl::KdTreeFLANN<Point>);
    this->tree_->setInputCloud(this->org_in_.makeShared());
    ROS_INFO("build tree done");

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    Eigen::VectorXf b;
    b.resize(36);
    for(int i=0; i<36; i++) b(i) = feature_out_.points[index].descriptor[i];
    return b;
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    Eigen::VectorXf b;
    b.resize(36);
    for(int i=0; i<36; i++) b(i) = feature_in_.points[index].descriptor[i];
    return b;
  }
};

#include "impl/narf.hpp"

#endif /* NARF_H_ */
