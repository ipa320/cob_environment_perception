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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 02/2012
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

#ifndef __COB_3D_MAPPING_FEATURES_SEGMENTATION_NODELET_H__
#define __COB_3D_MAPPING_FEATURES_SEGMENTATION_NODELET_H__


// ROS includes
#include <pcl_ros/pcl_nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// Package includes
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_mapping_features/organized_curvature_estimation_omp.h"
#include "cob_3d_mapping_features/extended_segmentation.h"

namespace cob_3d_mapping_features
{
  class SegmentationNodelet : public pcl_ros::PCLNodelet
  {
    typedef pcl::PointXYZRGB PointT;
    //typedef pcl::PointCloud<PointT> PointCloud;

  public:
    SegmentationNodelet() : it_(nh_)
      , one_()
      , oce_()
      , seg_()
      , colored_(new pcl::PointCloud<PointT>)
      , normals_(new pcl::PointCloud<pcl::Normal>)
      , pc_(new pcl::PointCloud<pcl::PrincipalCurvatures>)
      , labels_(new pcl::PointCloud<PointLabel>)
      , clusters_()
      , cluster_list_()
    { }

    ~SegmentationNodelet() 
    { }


  protected:    
    virtual void 
    onInit();

    void
    received_cloud_cb(const pcl::PointCloud<PointT>::ConstPtr& cloud);

    //boost::mutex mutex_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    
    OrganizedNormalEstimationOMP<PointT, pcl::Normal, PointLabel> one_;
    OrganizedCurvatureEstimationOMP<PointT, pcl::Normal, PointLabel, pcl::PrincipalCurvatures> oce_;
    ExtendedSegmentation<pcl::PointXYZRGB,pcl::Normal,pcl::PrincipalCurvatures,PointLabel> seg_;

    pcl::PointCloud<PointT>::Ptr colored_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pc_;
    pcl::PointCloud<PointLabel>::Ptr labels_;
    std::vector<pcl::PointIndices> clusters_;
    std::vector<cob_3d_mapping_common::Cluster> cluster_list_;
    cv::Mat segmented_;

  };

  
  
}


#endif  //__COB_3D_MAPPING_FEATURES_SEGMENTATION_NODELET_H__
