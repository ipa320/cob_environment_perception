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
 * feature_container.h
 *
 *  Created on: Nov 8, 2011
 *      Author: goa-jh
 */

#ifndef FEATURE_CONTAINER_H_
#define FEATURE_CONTAINER_H_


class FeatureContainerInterface
{
public:
  virtual bool isValid () = 0;
  virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                           std::vector<float> &distances) = 0;
};

template <typename FeatureType>
class FeatureContainer : public FeatureContainerInterface
{
public:
  typedef typename pcl::PointCloud<FeatureType>::ConstPtr FeatureCloudConstPtr;
  typedef typename pcl::KdTree<FeatureType> KdTree;
  typedef typename pcl::KdTree<FeatureType>::Ptr KdTreePtr;
  typedef boost::function<int (const pcl::PointCloud<FeatureType> &, int, std::vector<int> &,
                               std::vector<float> &)> SearchMethod;

  FeatureContainer () : k_(0), radius_(0) {}

  void setSourceFeature (const FeatureCloudConstPtr &source_features)
  {
    source_features_ = source_features;
  }

  FeatureCloudConstPtr getSourceFeature ()
  {
    return (source_features_);
  }

  void setTargetFeature (const FeatureCloudConstPtr &target_features)
  {
    target_features_ = target_features;
    if (tree_)
    {
      tree_->setInputCloud (target_features_);
    }
  }

  FeatureCloudConstPtr getTargetFeature ()
  {
    return (target_features_);
  }

  void setRadiusSearch (KdTreePtr tree, float r)
  {
    tree_ = tree;
    radius_ = r;
    k_ = 0;
    if (target_features_)
    {
      tree_->setInputCloud (target_features_);
    }
  }

  void setKSearch (KdTreePtr tree, int k)
  {
    tree_ = tree;
    k_ = k;
    radius_ = 0.0;
    if (target_features_)
    {
      tree_->setInputCloud (target_features_);
    }
  }

  virtual bool isValid ()
  {
    if (!source_features_ || !target_features_ || !tree_)
    {
      return (false);
    }
    else
    {
      return (source_features_->points.size () > 0 &&
          target_features_->points.size () > 0 &&
          (k_ > 0 || radius_ > 0.0));
    }
  }

  virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                           std::vector<float> &distances)
  {
    if (k_ > 0)
    {
      correspondence_indices.resize (k_);
      distances.resize (k_);
      tree_->nearestKSearch (*source_features_, index, k_, correspondence_indices, distances);
    }
    else
    {
      tree_->radiusSearch (*source_features_, index, radius_, correspondence_indices, distances);
    }
  }

private:
  FeatureCloudConstPtr source_features_, target_features_;
  KdTreePtr tree_;
  SearchMethod search_method_;
  int k_;
  double radius_;
};

template <typename Point>
class FeatureContainerInterface_Euclidean : public FeatureContainerInterface
{
protected:
  bool build_;
  float radius2_;
  pcl::PointCloud<Point> org_in_, org_out_;
  boost::shared_ptr<pcl::KdTree<Point> > tree_;
public:

  FeatureContainerInterface_Euclidean():
    build_(false), radius2_(0.05)
  {
  }

  void setSearchRadius(float v) {radius2_ = v;}

  virtual void build(const pcl::PointCloud<Point> &src, const pcl::PointCloud<Point> &tgt) {
    org_in_=src;
    org_out_=tgt;

    tree_.reset (new pcl::KdTreeFLANN<Point>);
    if(org_in_.size()>0) tree_->setInputCloud(org_in_.makeShared());

    build_ = hidden_build();
  }

  virtual bool isValid () {return  build_;}
  virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                           std::vector<float> &distances) {
    correspondence_indices.clear();
    distances.clear();

    int num_k2_=2;
    float border_=111;

    std::vector<int> inds;
    std::vector<float> dis;
    tree_->radiusSearch(org_out_.points[index], radius2_, inds, dis);
    if(inds.size()<1)
      tree_->nearestKSearch(org_out_.points[index], num_k2_, inds, dis);

    if(inds.size()<1)
      return;
    float min_dis=border_*border_;
    Eigen::VectorXf b = getFeatureOut(index);
    int mi=inds[0];
    for(int i=0; i<inds.size(); i++) {
      Eigen::VectorXf a = getFeatureIn(inds[i]);
      if( (a-b).squaredNorm()*(a-b).squaredNorm()*(org_in_.points[inds[i]].getVector3fMap()-org_out_.points[index].getVector3fMap()).squaredNorm()<min_dis ) {
        min_dis = (a-b).squaredNorm()*(a-b).squaredNorm()*(org_in_.points[inds[i]].getVector3fMap()-org_out_.points[index].getVector3fMap()).squaredNorm();
        mi = inds[i];
      }
    }
    if(mi!=-1)
      correspondence_indices.push_back(mi);

    for(int i=0; i<correspondence_indices.size(); i++) {
      distances.push_back( (org_in_.points[correspondence_indices[i]].getVector3fMap()-org_out_.points[index].getVector3fMap()).squaredNorm() );
    }
  }

  virtual Eigen::VectorXf getFeatureOut(const int)=0;
  virtual Eigen::VectorXf getFeatureIn(const int)=0;
  virtual bool hidden_build()=0;
};

#endif /* FEATURE_CONTAINER_H_ */
