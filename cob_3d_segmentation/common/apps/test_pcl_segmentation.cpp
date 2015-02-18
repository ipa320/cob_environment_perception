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
 * \date Date of creation: 05/2012
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

#if PCL_MINOR_VERSION >= 6

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <cob_3d_mapping_common/stop_watch.h>
#include "cob_3d_mapping_common/point_types.h"

using namespace pcl;

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Please provide an input pcd file" << std::endl;
    return 0;
  }

  PCDReader r;
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  float* distance_map;
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);
  PointCloud<Label>::Ptr l2(new PointCloud<Label>);

  r.read( argv[1], *p);

  PrecisionStopWatch t;
  std::cout << "Computing Normals..." << std::endl;
  t.precisionStart();
  /*search::OrganizedNeighbor<PointXYZRGB>::Ptr tree(new search::OrganizedNeighbor<PointXYZRGB>);
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setSearchMethod(tree);
  ne.setKSearch(200);
  ne.setInputCloud(p);
  ne.compute(*n);*/
  IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.02f);
  //ne.setDepthDependentSmoothing(true);
  ne.setNormalSmoothingSize(40.0f);
  ne.setInputCloud(p);
  ne.compute(*n);
  distance_map = ne.getDistanceMap();
  std::cout << "Took: " << t.precisionStop() << std::endl;

  std::cout << "Computing segments..." << std::endl;
  t.precisionStart();
  std::vector<PlanarRegion<PointXYZRGB>, Eigen::aligned_allocator<PlanarRegion<PointXYZRGB> > > regions;
  std::vector<ModelCoefficients> coef;
  std::vector<PointIndices> inlier_indices, label_indices, boundary_indices;
  OrganizedMultiPlaneSegmentation<PointXYZRGB, Normal, Label> omps;
  EdgeAwarePlaneComparator<PointXYZRGB, Normal>::Ptr comparator(new EdgeAwarePlaneComparator<PointXYZRGB, Normal>(distance_map));
  omps.setComparator(comparator);
  omps.setInputCloud(p);
  omps.setInputNormals(n);
  omps.setMinInliers(100);
  omps.setAngularThreshold(5.0f/180.0f*M_PI);
  omps.setMaximumCurvature(0.1); // default 0.001
  omps.setDistanceThreshold(0.02f);
  omps.segmentAndRefine(regions, coef, inlier_indices, l2, label_indices, boundary_indices);
  std::cout << "Took: " << t.precisionStop() << std::endl;
  std::cout << "Regions: " << regions.size() << std::endl;
  std::cout << "Coefs: " << coef.size() << std::endl;
  std::cout << "Inlier: " << inlier_indices.size() << std::endl;
  std::cout << "Label: " << label_indices.size() << std::endl;
  std::cout << "Boundary: " << boundary_indices.size() << std::endl;
  std::cout << "Cloud: " << l2->size() << std::endl;

  std::map<int, int> colors;
  for (size_t i = 0; i < regions.size(); i++)
  {
    uint8_t r = rand() % 255;
    uint8_t g = rand() % 255;
    uint8_t b = rand() % 255;
    colors[i] = (r << 16 | b << 8 | b);
    std::cout << "[" << i << "] => " << colors[i] << std::endl;
  }

  for (size_t i=0; i<inlier_indices.size(); ++i)
  {
    for (size_t j=0; j<inlier_indices[i].indices.size(); ++j)
    {
      (*p)[ inlier_indices[i].indices[j] ].rgba = colors[ i ];
    }
  }

  visualization::PCLVisualizer v;
  visualization::PointCloudColorHandlerRGBField<PointXYZRGB> col_hdl (p);


  int v1(0);
  v.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  v.addPointCloud<PointXYZRGB>(p, col_hdl, "segmented1");


  int v2(0);
  v.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  v.addPointCloudNormals<PointXYZRGB, Normal>(p, n, 10, 0.02, "boundary_normals", v2);
  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  return 0;
}


#else
// Better: check in CMakeLists for pcl version!
#include <iostream>
int main (int argc, char** argv)
{
  std::cout << "not supported by PCL older than 1.7" << std::endl;
}

#endif
