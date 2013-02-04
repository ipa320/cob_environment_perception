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
 * \date Date of creation: 02/2013
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

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/search/organized.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>

// stack includes:
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"


typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal NormalT;
typedef pcl::Label LabelT;

class PerformanceTester
{
public:
  PerformanceTester()
    : n(new pcl::PointCloud<NormalT>)
    , l(new pcl::PointCloud<LabelT>)
  {
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(40.0f);

    omps.setAngularThreshold(5.0f/180.0f*M_PI);
    omps.setMaximumCurvature(0.1); // default 0.001
    omps.setDistanceThreshold(0.02f);
    omps.setMinInliers(100);
  }

  void
  computeForOneCloud(pcl::PointCloud<PointT>::ConstPtr cloud)
  {
    PrecisionStopWatch t;
    t.precisionStart();
    ne.setInputCloud(cloud);
    ne.compute(*n);
    distance_map = ne.getDistanceMap();
    std::cout << t.precisionStop() << " | ";

    t.precisionStart();
    comparator.reset(new pcl::EdgeAwarePlaneComparator<PointT, NormalT>(distance_map));
    omps.setComparator(comparator);
    omps.setInputCloud(cloud);
    omps.setInputNormals(n);
    omps.segmentAndRefine(regions, coef, inlier_indices, l, label_indices, boundary_indices);
    std::cout << t.precisionStop() << std::endl;

    coef.clear();
    regions.clear();
    inlier_indices.clear();
    label_indices.clear();
    boundary_indices.clear();
    l.reset(new pcl::PointCloud<LabelT>);
  }

private:
  pcl::PointCloud<NormalT>::Ptr n;
  pcl::PointCloud<pcl::Label>::Ptr l;
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> coef;
  std::vector<pcl::PointIndices> inlier_indices;
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;

  pcl::IntegralImageNormalEstimation<PointT, NormalT> ne;
  float* distance_map;
  pcl::OrganizedMultiPlaneSegmentation<PointT, NormalT, LabelT> omps;
  pcl::EdgeAwarePlaneComparator<PointT, NormalT>::Ptr comparator;

};

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Please provide a folder containing multiple pcd files" << std::endl;
    return 0;
  }

  PerformanceTester pt;
  pcl::PCDReader r;
  std::vector<std::string> files;
  pcl::getAllPcdFilesInDirectory(argv[1], files);
  std::vector<pcl::PointCloud<PointT>::Ptr> clouds;
  for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); ++it)
  {
    clouds.push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));
    r.read( argv[1] + *it, *(clouds.back()) );
  }

  int i = 0;
  while (i < 20)
  {
    std::vector<pcl::PointCloud<PointT>::Ptr>::iterator p = clouds.begin();
    std::cout << "Run #"<< i << std::endl;
    while(p != clouds.end())
    {
      pt.computeForOneCloud(*p);
      ++p;
    }
    ++i;
  }

  return 0;
}
