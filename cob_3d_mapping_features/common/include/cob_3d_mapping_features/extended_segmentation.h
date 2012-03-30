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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 10/2011
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

#ifndef __EXTENDED_SEGMENTATION_H__
#define __EXTENDED_SEGMENTATION_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/cluster.h>

namespace cob_3d_mapping_features
{
  template <typename PointT, typename PointNT, typename PointCT, typename PointOutT>
  class ExtendedSegmentation
  {
  public:
    typedef pcl::PointCloud<PointT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef pcl::PointCloud<PointNT> NormalCloudIn;
    typedef typename NormalCloudIn::Ptr NormalCloudInPtr;
    typedef typename NormalCloudIn::ConstPtr NormalCloudInConstPtr;

    typedef pcl::PointCloud<PointCT> CurvatureCloudIn;
    typedef typename CurvatureCloudIn::Ptr CurvatureCloudInPtr;
    typedef typename CurvatureCloudIn::ConstPtr CurvatureCloudInConstPtr;

    typedef pcl::PointCloud<PointOutT> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

    typedef std::vector<cob_3d_mapping_common::Cluster> ClusterList;

  public:
    /** \brief Empty constructor. */
    ExtendedSegmentation () : color_tab_()
    { 
      color_tab_.reserve(2052);
      color_tab_.push_back(cv::Vec3b(0, 255, 0)); // undef
      color_tab_.push_back(cv::Vec3b(255, 255, 255)); // nan
      color_tab_.push_back(cv::Vec3b(0, 0, 255)); // border
      color_tab_.push_back(cv::Vec3b(0, 0, 255)); // edge
      color_tab_.push_back(cv::Vec3b(255, 200, 0)); // first
      color_tab_.push_back(cv::Vec3b(0, 200, 255)); // second
      color_tab_.push_back(cv::Vec3b(0, 200, 0)); // third
      for (size_t i=0; i<2048; ++i)
      {
	uchar b = cv::theRNG().uniform(0, 255);
	uchar g = cv::theRNG().uniform(0, 255);
	uchar r = cv::theRNG().uniform(0, 255);
	color_tab_.push_back(cv::Vec3b(b, g, r));
      }
    };

    inline void
    setInput(const PointCloudInConstPtr &points) { surface_ = points; }

    inline void
    setInputNormals(const NormalCloudInConstPtr &normals) { normals_ = normals; }

    inline void
    setInputCurvatures(const CurvatureCloudInConstPtr &curvatures) { curvatures_ = curvatures; }

    void
    propagateWavefront(const LabelCloudPtr& labels,
		       ClusterList& cluster_out);

    void
    analyseClusters(ClusterList& cluster_out);

    void
    getColoredCloud(ClusterList& cluster_list,
		    pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud);

    void
    getColoredCloudByType(ClusterList& cluster_list,
			  pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud);

  protected:
    PointCloudInConstPtr surface_;
    NormalCloudInConstPtr normals_;
    CurvatureCloudInConstPtr curvatures_;
    std::vector<cv::Vec3b> color_tab_;

    struct Coords
    {
    public:
    Coords(int u_in, int v_in, float angle) : u(u_in), v(v_in), angle(angle)
	{ }
      
      int u;
      int v;
      float angle;
    };

    
  };
}

#endif  //#ifndef __EXTENDED_SEGMENTATION_H__


