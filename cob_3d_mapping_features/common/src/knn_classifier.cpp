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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
 * ToDo:
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

/*
 * knn_classifier.cpp
 *
 *  Created on: 04.10.2011
 *      Author: goa-sf
 */

#include <fstream>
#include <boost/foreach.hpp>

#include <cob_3d_mapping_features/knn_classifier.h>

#include <flann/algorithms/dist.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cob_3d_mapping_features;

template<typename PointT>
KNNClassifier<PointT>::KNNClassifier() :
  k_(1)
{ } 

template<typename PointT>
KNNClassifier<PointT>::~KNNClassifier()
{ }

/*********************************************************************
 *
 * !! The following only works with a modified PCL !!
 * New instantiations of the KdTreeFLANN class are needed.
 *
 *********************************************************************/
/*
template<typename PointT> void 
KNNClassifier<PointT>::setTrainingFeatures(const PointCloudPtr &features, string &distanceMetric)
{
  //pcl::KdTreeFLANN<PointT, flann::L1<float> > tree;
  typename pcl::CustomPointRepresentation<PointT>::Ptr cpr (new pcl::CustomPointRepresentation<PointT> (INT_MAX, 0));
  if(distanceMetric == "L2")
  {
    tree_.reset(new pcl::KdTreeFLANN<PointT, flann::L2_Simple<float> >());
    distanceMetric_ = distanceMetric;
  }
  else if (distanceMetric == "L1")
  {
    tree_.reset(new pcl::KdTreeFLANN<PointT, flann::L1<float> >());
    distanceMetric_ = distanceMetric;
  }
  else if (distanceMetric == "HIK")
  {
    tree_.reset(new pcl::KdTreeFLANN<PointT, flann::HistIntersectionDistance<float> >());
    distanceMetric_ = distanceMetric;
  }
  else if (distanceMetric == "ChiSquare")
  {
    tree_.reset(new pcl::KdTreeFLANN<PointT, flann::ChiSquareDistance<float> >());
    distanceMetric_ = distanceMetric;
  }
  else if (distanceMetric == "Hellinger")
  {
    tree_.reset(new pcl::KdTreeFLANN<PointT, flann::HellingerDistance<float> >());
    distanceMetric_ = distanceMetric;
  }
  else
  {
    cout << "[KNNClassifier::setTrainingFeatures()]: \"" << distanceMetric << "\" not supported!\n";
    cout << "Selected default: L2" << endl;
    tree_.reset(new pcl::KdTreeFLANN<PointT>);
    distanceMetric_ = "L2";
  }
  tree_->setPointRepresentation(cpr);
  tree_->setInputCloud(features);
}
*/

/*********************************************************************
 *
 * Use the following for an unmodified version of PCL
 *
 *********************************************************************/

template<typename PointT> void 
KNNClassifier<PointT>::setTrainingFeatures(const PointCloudPtr &features, string &distanceMetric)
{
  typename pcl::CustomPointRepresentation<PointT>::Ptr cpr (new pcl::CustomPointRepresentation<PointT> (INT_MAX, 0));
  tree_.reset(new pcl::KdTreeFLANN<PointT>);
  distanceMetric_ = "L2";
  tree_->setPointRepresentation(cpr);
  tree_->setInputCloud(features);
}

template<typename PointT> void 
KNNClassifier<PointT>::setTrainingLabels(const vector<int> &labels)
{
  labels_ = labels;
  labels_max_ = 7;//*max_element(labels_.begin(), labels_.end());
}

template<typename PointT> void 
KNNClassifier<PointT>::setKNeighbors(int k)
{
  k_ = k;
}

template<typename PointT> int 
KNNClassifier<PointT>::saveTrainingData(string &features_file_name, string &labels_file_name)
{
  PointCloudConstPtr training_features = tree_->getInputCloud();
  if (labels_.size() == training_features->size())
  {
    if (int res = pcl::io::savePCDFile(features_file_name.c_str(), *training_features) == -1)
      return res;
    ofstream f (labels_file_name.c_str());
    f << distanceMetric_ << endl;
    BOOST_FOREACH (int i, labels_)
      f << i << endl;
    return 0;
  }
  return -1;
}

template<typename PointT> int 
KNNClassifier<PointT>::loadTrainingData(string &features_file_name, string &labels_file_name)
{
  PointCloudPtr cloud(new pcl::PointCloud<PointT>);
  if (int res = pcl::io::loadPCDFile(features_file_name.c_str(), *cloud) == -1)
    return res;
  vector<int> labels;
  ifstream f (labels_file_name.c_str());
  string line, metric;
  getline(f, metric);

  while(getline(f, line))
  {
    if (line.size() > 0)
      labels.push_back(atoi(line.c_str()));
  }
  if (labels.size() != cloud->size())
    return -1;
  setTrainingLabels(labels);
  setTrainingFeatures(cloud, metric);
  return 0;
}

template<typename PointT> int 
KNNClassifier<PointT>::classify(const PointT &p_q)
{
  vector<int> label_counter(labels_max_, 0);
  vector<int> k_indices;
  vector<float> k_distances;
  tree_->nearestKSearch(p_q, k_, k_indices, k_distances);
  BOOST_FOREACH (int i, k_indices)
    label_counter[labels_[i]]++;
//  int max_idx = *max_element(label_counter.begin(), label_counter.end());
  int max_label_counter = 0;
  for (int i=0; i<labels_max_; ++i)
    max_label_counter = max(label_counter[i], max_label_counter);
  for (int i=0; i<labels_max_; ++i)
  {
    if(label_counter[i] == max_label_counter)
      return i;
  }
  return (-1);
}

template class KNNClassifier<pcl::PointXYZ>;
template class KNNClassifier<pcl::PointXYZRGBA>;
template class KNNClassifier<pcl::FPFHSignature33>;
