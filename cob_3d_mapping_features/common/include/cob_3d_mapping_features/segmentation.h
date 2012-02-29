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

#ifndef __SEGMENTATION_H__
#define __SEGMENTATION_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <opencv2/core/core.hpp>
#include <cob_3d_mapping_common/point_types.h>

// define indices to access labels list:
#define I_NAN    1
#define I_BORDER 2
#define I_EDGE   3
#define I_PLANE  4
#define I_CYL    5
#define I_SPHERE 6

namespace cob_3d_mapping_features
{

  struct Coords
  {
    int u;
    int v;
    bool c_gap;

    Coords(int u_in, int v_in, bool c_gap_in)
      {
	u = u_in;
	v=  v_in;
	c_gap = c_gap_in;
      }
  };

  class Segmentation
  {
  public:
    /** \brief Empty constructor. */
    Segmentation () { };

    int searchForNeighbors (
      pcl::PointCloud<PointLabel>::Ptr& cloud_in,
      int col, int row,
      double radius,
      std::vector<int>& indices_ul,
      std::vector<int>& indices_ur,
      std::vector<int>& indices_lr,
      std::vector<int>& indices_ll,
      bool& gap_l, bool& gap_r, bool& gap_a, bool& gap_d);
    bool isStopperInNeighbors(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<int>& indices);
    void propagateWavefront2(pcl::PointCloud<PointLabel>::Ptr& cloud_in);
    void getClusterIndices(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<pcl::PointIndices>& cluster_indices, cv::Mat& seg_img);

  };
}

#endif  //#ifndef __SEGMENTATION_H__


