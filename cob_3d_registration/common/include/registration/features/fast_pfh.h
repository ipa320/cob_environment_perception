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
 * fast_pfh.h
 *
 *  Created on: Nov 14, 2011
 *      Author: goa-jh
 */

#ifndef FAST_PFH_H_
#define FAST_PFH_H_

#include "../feature_container.h"
#include <pcl/features/fpfh.h>
#ifdef GICP_ENABLE
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#else
#include <pcl/kdtree/kdtree.h>
#endif

template<typename Point>
class Feature_FPFH : public FeatureContainerInterface_Euclidean<Point>
{
  float radius_;

  pcl::PointCloud<pcl::FPFHSignature33> feature_in_, feature_out_;

public:
  Feature_FPFH():
    radius_(0.1)
  {
  }

  void setFPFHRadius(float v) {radius_ = v;}

  virtual bool hidden_build() {

    ROS_INFO("calc fpfh for source");
    {
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree <Point>);
#else
	  #ifdef PCL_VERSION_COMPARE
		boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
	  #else
        boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
	  #endif
#endif
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

      pcl::NormalEstimation<Point, pcl::Normal> ne;
      ne.setSearchMethod (tree);
      ne.setInputCloud (this->org_in_.makeShared());
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.05);

      // Compute the features
      ne.compute (*normals);

      // Create the FPFH estimation class, and pass the input dataset+normals to it
      pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud (this->org_in_.makeShared());
      fpfh.setInputNormals (normals);

      fpfh.setSearchMethod (tree);
      fpfh.setRadiusSearch (radius_);

      // Compute the features
      fpfh.compute (feature_in_);
    }

    ROS_INFO("calc fpfh for target");
    {
#ifdef PCL_VERSION_COMPARE
  #ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#  else
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
  #endif
#else
  #ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  #else
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  #endif
#endif
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

      pcl::NormalEstimation<Point, pcl::Normal> ne;
      ne.setSearchMethod (tree);
      ne.setInputCloud (this->org_out_.makeShared());
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.1);

      // Compute the features
      ne.compute (*normals);

      // Create the FPFH estimation class, and pass the input dataset+normals to it
      pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud (this->org_out_.makeShared());
      fpfh.setInputNormals (normals);

      fpfh.setSearchMethod (tree);
      fpfh.setRadiusSearch (radius_);

      // Compute the features
      fpfh.compute (feature_out_);
    }

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    Eigen::VectorXf b;
    b.resize(33);
    for(int i=0; i<33; i++) b(i) = feature_out_.points[index].histogram[i];
    return b;
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    Eigen::VectorXf b;
    b.resize(33);
    for(int i=0; i<33; i++) b(i) = feature_in_.points[index].histogram[i];
    return b;
  }

};


#endif /* FAST_PFH_H_ */
