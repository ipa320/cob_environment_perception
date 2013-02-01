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
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
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

#ifndef __SIMPLE_SEGMENTATION_NODELET_H__
#define __SIMPLE_SEGMENTATION_NODELET_H__

// PCL includes
//#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>

// ROS includes
#include <nodelet/nodelet.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_segmentation/segmentation_nodeletConfig.h>


// Package includes
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_segmentation/impl/fast_segmentation.hpp"
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>

namespace cob_3d_segmentation
{
  class SimpleSegmentationNodelet : public nodelet::Nodelet
  {
    public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef pcl::PointCloud<PointLabel> LabelCloud;
    typedef FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterPtr ClusterPtr;

    public:
    SimpleSegmentationNodelet()
      : one_()
      , seg_()
      , down_(new PointCloud)
      , segmented_(new PointCloud)
      , normals_(new NormalCloud)
      , labels_(new LabelCloud)
      , centroid_passthrough_(5.0f)
      , min_cluster_size_(100)
      , filter_(false)
      , downsample_(false)
    { }

    ~SimpleSegmentationNodelet()
    { }

    protected:
    void onInit();
    void configCallback(cob_3d_segmentation::segmentation_nodeletConfig& config, uint32_t levels);

    void receivedCloudCallback(PointCloud::ConstPtr cloud);


    ros::NodeHandle nh_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_segmented_;
    ros::Publisher pub_shape_array_;

    boost::shared_ptr<dynamic_reconfigure::Server<cob_3d_segmentation::segmentation_nodeletConfig> > config_server_;

    cob_3d_mapping_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
    FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel> seg_;
    PolygonExtraction pe_;

    PointCloud::Ptr down_;
    PointCloud::Ptr segmented_;
    NormalCloud::Ptr normals_;
    LabelCloud::Ptr labels_;

    float centroid_passthrough_;
    int min_cluster_size_;
    bool filter_;
    bool downsample_;
  };
}


#endif
