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
  typedef pcl::PointXYZRGB Point;

  TableObjectCluster()
  {
    height_min_ = -0.5;
    height_max_ = -0.03;
    min_cluster_size_ = 30;
    cluster_tolerance_ = 0.03;

  };
  ~TableObjectCluster() {};

  /**
   * @brief extracts objects at top of a plane
   *
   * extracts objects at top of a plane using a prism
   *
   * @param pc_in input point cloud
   * @param hull hull describing plane
   * @param pc_roi output point cloud containing extracted objects
   *
   * @return nothing
   */
  void
  extractTableRoi(pcl::PointCloud<Point>::Ptr& hull,
                  pcl::PointIndices& pc_roi);

  /**
   * @brief extracts objects at top of a plane
   *
   * extracts objects at top of a plane using distance
   *
   * @param pc_in input point cloud
   * @param hull hull describing plane
   * @param plane_coeffs coefficients describing plane
   * @param pc_roi output point cloud containing extracted objects
   *
   * @return nothing
   */
  void
  extractTableRoi2(pcl::PointCloud<Point>::Ptr& pc_in,
                                      pcl::PointCloud<Point>::Ptr& hull,
                                      Eigen::Vector4f& plane_coeffs,
                                      pcl::PointCloud<Point>& pc_roi);

  /**
   * @brief removes known objects by bounding box
   *
   * removes known objects by bounding box
   *
   * @param pc_roi input point cloud with objects
   * @param bounding_boxes bounding boxes of known objects
   * @param pc_roi_red output point cloud without known objects
   *
   * @return nothing
   */
  void
  removeKnownObjects(pcl::PointCloud<Point>::Ptr& pc_roi,
                     std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > >& bounding_boxes,
                     pcl::PointCloud<Point>& pc_roi_red);

  /**
   * @brief removes known objects by bounding box
   *
   * removes known objects by bounding box
   *
   * @param pc_roi input point cloud with objects
   * @param bounding_boxes bounding boxes of known objects
   * @param pc_roi_red output point cloud without known objects
   *
   * @return nothing
   */
  void
  calculateBoundingBoxes(pcl::PointIndices::Ptr& pc_roi,
                         std::vector<pcl::PointCloud<Point>::Ptr >& object_clusters,
                     std::vector<pcl::PointCloud<pcl::PointXYZ> >& bounding_boxes);


  void
  setInputCloud(const pcl::PointCloud<Point>::Ptr&  cloud) {input_ = cloud;}

  /**
   * @brief sets parameters for filtering
   *
   * sets parameters for filtering
   *
   * @param height_min
   * @param height_max
   *
   * @return nothing
   */
  void
  setPrismHeight(double height_min, double height_max)
  {
    height_min_ = height_min;
    height_max_ = height_max;
  }

  /**
   * @brief sets parameters for filtering
   *
   * sets parameters for filtering
   *
   * @param min_cluster_size
   * @param cluster_tolerance
   *
   * @return nothing
   */
  void
  setClusterParams(int min_cluster_size, double cluster_tolerance)
  {
    min_cluster_size_ = min_cluster_size;
    cluster_tolerance_ = cluster_tolerance;
  }

protected:
  pcl::PointCloud<Point>::Ptr input_;
  double height_min_;           /// paramter for object detection
  double height_max_;           /// paramter for object detection
  int min_cluster_size_;        /// paramter for object detection
  double cluster_tolerance_;    /// paramter for object detection

};

#endif /* __TABLE_OBJECT_CLUSTER_H__ */
