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
 * ROS package name: cob_3d_segmentation
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
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

#ifndef __SEGMENTATION_ALL_IN_ONE_NODELET_H__
#define __SEGMENTATION_ALL_IN_ONE_NODELET_H__


// ROS includes
#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_segmentation/segmentation_nodeletConfig.h>

// PCL includes
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>

// Package includes
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_segmentation/depth_segmentation.h"
#include "cob_3d_segmentation/cluster_classifier.h"
#include "cob_3d_segmentation/polygon_extraction/polygon_types.h"
#include "cob_3d_segmentation/polygon_extraction/polygon_extraction.h"


namespace cob_3d_segmentation
{
  class SegmentationAllInOneNodelet : public pcl_ros::PCLNodelet
  {
  public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef pcl::PointCloud<PointLabel> LabelCloud;
    typedef PredefinedSegmentationTypes ST;


  public:
    SegmentationAllInOneNodelet()
      : one_()
      , seg_()
      , graph_(new ST::Graph)
      , pe_()
      , segmented_(new PointCloud)
      , classified_(new PointCloud)
      , normals_(new NormalCloud)
      , labels_(new LabelCloud)
      , centroid_passthrough_(5.0f)
    { }

    ~SegmentationAllInOneNodelet()
    { }


  protected:
    void onInit();
    void configCallback(cob_3d_segmentation::segmentation_nodeletConfig& config, uint32_t level);

    void receivedCloudCallback(PointCloud::ConstPtr cloud);
    void publishShapeArray(ST::CH::Ptr cluster_handler, std::map<int,int>& objs, PointCloud::ConstPtr cloud);

    //boost::mutex mutex_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_segmented_;
    ros::Publisher pub_classified_;
    ros::Publisher pub_shape_array_;
    ros::Publisher pub_chull_;
    ros::Publisher pub_chull_dense_;

    boost::shared_ptr<dynamic_reconfigure::Server<cob_3d_segmentation::segmentation_nodeletConfig> > config_server_;

    cob_3d_mapping_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
    DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg_;
    ClusterClassifier<ST::CH, ST::Point, ST::Normal, ST::Label> cc_;
    ST::Graph::Ptr graph_;
    PolygonExtraction pe_;

    PointCloud::Ptr segmented_;
    PointCloud::Ptr classified_;
    NormalCloud::Ptr normals_;
    LabelCloud::Ptr labels_;


    float centroid_passthrough_;

    boost::mutex mutex_;
  };

}


#endif  //__COB_3D_MAPPING_FEATURES_SEGMENTATION_NODELET_H__
