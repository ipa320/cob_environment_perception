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
  extractPlanes(const pcl::PointCloud<Point>::ConstPtr& pc_in,
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

  void
  findClosestTable(std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                   std::vector<pcl::ModelCoefficients>& v_coefficients_plane,
                   Eigen::Vector3f& robot_pose,
                   unsigned int& idx);


protected:
  int ctr_;
  unsigned int min_cluster_size_;
  std::string file_path_;
  bool save_to_file_;
  PlaneConstraint plane_constraint_;

  pcl::EuclideanClusterExtraction<Point> cluster_;
  pcl::EuclideanClusterExtraction<Point> cluster_2_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ExtractIndices<Point> extract_;
  pcl::NormalEstimation<Point,pcl::Normal> normal_estimator_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConcaveHull<Point> chull_;

};

#endif //__PLANE_EXTRACTION_H__
