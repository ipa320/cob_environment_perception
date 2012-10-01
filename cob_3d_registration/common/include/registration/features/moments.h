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
 * moments.h
 *
 *  Created on: Nov 11, 2011
 *      Author: goa-jh
 */

#ifndef MOMENTS_H_
#define MOMENTS_H_

#include "../feature_container.h"
#include <pcl/features/moment_invariants.h>
#include <pcl/features/impl/moment_invariants.hpp>
#ifdef GICP_ENABLE
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#else
#include <pcl/kdtree/kdtree.h>
#endif

template<typename Point>
class Feature_MomentInvariants : public FeatureContainerInterface_Euclidean<Point>
{
  float radius_;

  pcl::PointCloud<pcl::MomentInvariants> feature_in_, feature_out_;

public:
  Feature_MomentInvariants():
    radius_(0.1)
  {
  }

  void setMomentRadius(float v) {radius_ = v;}

  virtual bool hidden_build() {

    ROS_INFO("calc moments for source");
    {
      pcl::MomentInvariantsEstimation<Point,pcl::MomentInvariants> est;
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#else
  #ifdef PCL_VERSION_COMPARE
    boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
  #else
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  #endif
#endif

      est.setInputCloud(this->org_in_.makeShared());
      est.setSearchMethod(tree);
      est.setRadiusSearch(radius_);

      est.compute(feature_in_);
    }

    ROS_INFO("calc moments for target");
    {
      pcl::MomentInvariantsEstimation<Point,pcl::MomentInvariants> est;
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#else
  #ifdef PCL_VERSION_COMPARE
    boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
  #else
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  #endif
#endif

      est.setInputCloud(this->org_out_.makeShared());
      est.setSearchMethod(tree);
      est.setRadiusSearch(radius_);

      est.compute(feature_out_);
    }

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    Eigen::Vector3f b;
    b(0) = feature_out_.points[index].j1;
    b(1) = feature_out_.points[index].j2;
    b(2) = feature_out_.points[index].j3;
    return b;
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    Eigen::Vector3f b;
    b(0) = feature_in_.points[index].j1;
    b(1) = feature_in_.points[index].j2;
    b(2) = feature_in_.points[index].j3;
    return b;
  }

};


#endif /* MOMENTS_H_ */
