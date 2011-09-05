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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2011
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

#ifndef __TABLE_OBJECT_CLUSTER_H__
#define __TABLE_OBJECT_CLUSTER_H__

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/StdVector>
#include <pcl/ModelCoefficients.h>


class TableObjectCluster
{
public:
  typedef pcl::PointXYZ Point;

  TableObjectCluster()
  {
    height_min_ = -0.5;
    height_max_ = -0.03;
    min_cluster_size_ = 30;
    cluster_tolerance_ = 0.03;

  };
  ~TableObjectCluster() {};

  void
  extractTableRoi(pcl::PointCloud<Point>::Ptr& pc_in,
                  pcl::PointCloud<Point>::Ptr& hull,
                  pcl::PointCloud<Point>& pc_roi);

  void
  extractTableRoi2(pcl::PointCloud<Point>::Ptr& pc_in,
                                      pcl::PointCloud<Point>::Ptr& hull,
                                      Eigen::Vector4f& plane_coeffs,
                                      pcl::PointCloud<Point>& pc_roi);

  void
  removeKnownObjects(pcl::PointCloud<Point>::Ptr& pc_roi,
                     std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > >& bounding_boxes,
                     pcl::PointCloud<Point>& pc_roi_red);

  void
  calculateBoundingBoxes(pcl::PointCloud<Point>::Ptr& pc_roi_red,
                     std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > >& bounding_boxes);

  void
  setPrismHeight(double height_min, double height_max)
  {
    height_min_ = height_min;
    height_max_ = height_max;
  }

  void
  setClusterParams(int min_cluster_size, double cluster_tolerance)
  {
    min_cluster_size_ = min_cluster_size;
    cluster_tolerance_ = cluster_tolerance;
  }

protected:
  double height_min_;
  double height_max_;
  int min_cluster_size_;
  double cluster_tolerance_;

};

#endif /* __TABLE_OBJECT_CLUSTER_H__ */
