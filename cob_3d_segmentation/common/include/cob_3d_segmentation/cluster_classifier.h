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
 * Date of creation: 04/2012
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

#ifndef __CLUSTER_CLASSIFIER_H__
#define __CLUSTER_CLASSIFIER_H__

#include "cob_3d_segmentation/cluster_handler.h"

#include "cob_3d_mapping_common/label_defines.h"

namespace cob_3d_segmentation
{
  template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT>
  class ClusterClassifier
  {
  public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<NormalT> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;
    typedef pcl::PointCloud<LabelT> LabelCloud;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

    typedef typename ClusterHandlerT::Ptr ClusterHdlPtr;
    typedef typename ClusterHandlerT::ClusterPtr ClusterPtr;
    typedef typename ClusterHandlerT::ClusterType ClusterType;

  public:
    ClusterClassifier() 
    { }

    ~ClusterClassifier()
    { }

    inline void setClusterHandler(ClusterHdlPtr clusters) { clusters_ = clusters; }
    inline void setPointCloudIn(PointCloudConstPtr points) { surface_ = points; }
    inline void setNormalCloudInOut(NormalCloudPtr normals) { normals_ = normals; }
    inline void setLabelCloudIn(LabelCloudConstPtr labels) { labels_ = labels; }

    void classify();
    void classifyOld();

    void mapUnusedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
    {
      uint32_t color = LBL_COR;
      for (std::vector<int>::iterator it = test.begin(); it != test.end(); ++it)
	points->points[*it].rgb = *reinterpret_cast<float*>(&color);
    }

    void mapPointClasses(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
    {
      uint32_t color = LBL_COR;
      for (std::vector<int>::iterator it = test.begin(); it != test.end(); ++it)
	points->points[*it].rgb = *reinterpret_cast<float*>(&color);

      color = LBL_PLANE;
      for (std::vector<int>::iterator it = test_plane.begin(); it != test_plane.end(); ++it)
	points->points[*it].rgb = *reinterpret_cast<float*>(&color);      

      color = LBL_CYL;
      for (std::vector<int>::iterator it = test_cyl.begin(); it != test_cyl.end(); ++it)
	points->points[*it].rgb = *reinterpret_cast<float*>(&color);

      color = LBL_SPH;
      for (std::vector<int>::iterator it = test_sph.begin(); it != test_sph.end(); ++it)
	points->points[*it].rgb = *reinterpret_cast<float*>(&color);
    }

  private:
    void recomputeClusterNormals(ClusterPtr c);
    void recomputeClusterNormals(ClusterPtr c, int w_size, int steps);
    bool computeClusterPointCurvature(int index, int r, int steps, 
				      float& pc_min, float& pc_max, Eigen::Vector3f& pc_min_direction);


    ClusterHdlPtr clusters_;
    LabelCloudConstPtr labels_;
    PointCloudConstPtr surface_;
    NormalCloudPtr normals_;
    std::vector<int> test;
    std::vector<int> test_cyl;
    std::vector<int> test_plane;
    std::vector<int> test_sph;
  };

}

#endif
