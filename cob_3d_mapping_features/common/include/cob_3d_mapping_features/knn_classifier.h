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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef COB_3D_MAPPING_FEATURES_KNN_CLASSIFIER_H_
#define COB_3D_MAPPING_FEATURES_KNN_CLASSIFIER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace cob_3d_mapping_features
{
  /*! @brief k-nearest-neighbor classifier */
  template<typename PointT> class KNNClassifier
  {
  public:
    /*! Empty constructor */
    KNNClassifier();
    /*! Empty destructor */
    ~KNNClassifier();

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
    typedef pcl::KdTree<PointT> KdTree;
    typedef boost::shared_ptr<KdTree> KdTreePtr;

    /*! 
     * @brief Set a point cloud containing the features samples
     *
     * @param[in] features input feature point cloud
     * @param[in] distanceMetric L2, L1, HIK, ChiSpuare, Hellinger
     *            at the moment only L2 works with the default PCL
     */
    void setTrainingFeatures(const PointCloudPtr &features, 
			     std::string &distanceMetric = "L2");

    /*!
     * @brief setTrainingLabels
     */
    void setTrainingLabels(const std::vector<int> &labels);

    /*!
     * @brief set the k neighbors to search for
     */
    void setKNeighbors(int k);

    /*!
     * @brief save the loaded training data and labels
     *
     * @param[in] features_file_name path where the features should be saved
     * @param[in] labels_file_name path where the labels should be saved
     */
    int saveTrainingData(std::string &features_file_name,
			 std::string &labels_file_name);

    /*!
     * @brief load the training data and labels
     *
     * @param[in] features_file_name path to features file
     * @param[in] labels_file_name path to labels file
     */
    int loadTrainingData(std::string &features_file_name,
			 std::string &labels_file_name);

    /*!
     * @brief classify a single point using the loaded training data
     *
     * @param[in] p_q a feature point to be classified
     * @return the predicted label
     */
    int classify(const PointT &p_q);

  protected:
    KdTreePtr tree_;
    std::vector<int> labels_;
    std::string distanceMetric_;
    int labels_max_;
    int k_;

//    int getDominantLabel(std::vector<int> 
  };
}

#endif
