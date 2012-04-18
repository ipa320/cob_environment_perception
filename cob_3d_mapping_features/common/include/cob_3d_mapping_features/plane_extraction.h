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

#ifndef __PLANE_EXTRACTION_H__
#define __PLANE_EXTRACTION_H__

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/StdVector>


enum PlaneConstraint {NONE, HORIZONTAL, VERTICAL};

//####################
//#### nodelet class ####
class PlaneExtraction
{
public:
  typedef pcl::PointXYZRGB Point;
  // Constructor
  PlaneExtraction();

  // Destructor
  ~PlaneExtraction()
  {
    /// void
  }

  void
  extractPlanes(const pcl::PointCloud<Point>::Ptr& pc_in,
                std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                std::vector<std::vector<pcl::Vertices> >& v_hull_polygons,
                std::vector<pcl::ModelCoefficients>& v_coefficients_plane);

  void
  saveHulls(pcl::PointCloud<Point>& cloud_hull,
            std::vector< pcl::Vertices >& hull_polygons,
            int plane_ctr);

  void
  setPlaneConstraint(PlaneConstraint constr)
  {
    plane_constraint_ = constr;
  }

  void
  setFilePath(std::string file_path)
  {
    file_path_ = file_path;
  }

  void
  setSaveToFile(bool save_to_file)
  {
    save_to_file_ = save_to_file;
  }

  inline void
  setClusteringParamClusterTolerance (double cluster_tolerance)
  {
    cluster_.setClusterTolerance (cluster_tolerance);
  }

  inline void
  setClusteringParamMinClusterSize (unsigned int min_cluster_size)
  {
    cluster_.setMinClusterSize (min_cluster_size);
  }

  inline void
  setClusteringParamMaxClusterSize (unsigned int max_cluster_size)
  {
    cluster_.setMaxClusterSize (max_cluster_size);
  }

  inline void
  setNormalEstimationParamRadius (double radius)
  {
    normal_estimator_.setRadiusSearch (radius);
  }

  inline void
  setSegmentationParamOptimizeCoefficients (bool optimize_coefficients)
  {
    seg_.setOptimizeCoefficients (optimize_coefficients);
  }

  inline void
  setSegmentationParamModelType (int model_type)
  {
    seg_.setModelType (model_type);
  }

  inline void
  setSegmentationParamMethodType (int method_type)
  {
    seg_.setModelType (method_type);
  }

  inline void
  setSegmentationParamNormalDistanceWeight (double normal_distance_weight)
  {
    seg_.setNormalDistanceWeight (normal_distance_weight);
  }

  inline void
  setSegmentationParamMaxIterations (int max_iterations)
  {
    seg_.setMaxIterations (max_iterations);
  }

  inline void
  setSegmentationParamDistanceThreshold (double threshold)
  {
    seg_.setDistanceThreshold (threshold);
  }
  void
  findClosestTable(std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                   std::vector<pcl::ModelCoefficients>& v_coefficients_plane,
                   Eigen::Vector3f& robot_pose,
                   unsigned int& idx);


protected:
  /**
   * @brief extract Euclidean clusters from point cloud data
   *
   * @param pc_in input point cloud
   * @param clusters indices of the found clusters
   *
   * @return nothing
   */
  void
  extractClusters (const pcl::PointCloud<Point>::Ptr& pc_in, std::vector<pcl::PointIndices>& clusters);

  /**
   * @brief check whether given plane is valid or not
   *
   * @param coefficients_plane pointer to coefficients of the plane to be evaluated
   *
   * @return nothing
   */
  bool
  isValidPlane(const pcl::ModelCoefficients& coefficients_plane);

  /**
   * @brief write point cloud data to a file
   *
   * @param dominant_plane_ptr pointer to point cloud specifying dominant plane
   *
   * @return nothing
   */
  void
  dumpToPCDFileAllPlanes (pcl::PointCloud<Point>::Ptr dominant_plane_ptr);

  int ctr_;

  std::string file_path_;
  bool save_to_file_;
  PlaneConstraint plane_constraint_;

  pcl::PointCloud<Point> extracted_planes_;
  //clustering parameters
  double cluster_tolerance_;unsigned int min_cluster_size_;unsigned int max_cluster_size_;

  //Normal Estimation parameters
  double radius_;

  //segmentation parameters
  bool optimize_coefficients_;
  double normal_distance_weight_;int max_iterations_;
  double distance_threshold_;

  pcl::EuclideanClusterExtraction<Point> cluster_;
  pcl::EuclideanClusterExtraction<Point> cluster_2_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ExtractIndices<Point> extract_;
  pcl::NormalEstimation<Point, pcl::Normal> normal_estimator_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConcaveHull<Point> chull_;

};

#endif //__PLANE_EXTRACTION_H__
