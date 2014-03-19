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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
//#include "cob_3d_mapping_features/fast_edge_estimation_3d_omp.h"
#include "cob_3d_features/organized_normal_estimation_omp.h"
#include "cob_3d_features/organized_normal_estimation.h"

using namespace pcl;

void determinePlaneNormal(PointCloud<PointXYZ>::Ptr& p, Eigen::Vector3f& normal)
{
  SACSegmentation<PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (p);
  ModelCoefficients coefficients_plane;
  pcl::PointIndices inliers_plane;
  seg.segment (inliers_plane, coefficients_plane);
  normal(0) = coefficients_plane.values[0];
  normal(1) = coefficients_plane.values[1];
  normal(2) = coefficients_plane.values[2];
}

int main(int argc, char** argv)
{
  PointCloud<PointXYZ>::Ptr p(new PointCloud<PointXYZ>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  pcl::PointCloud<pcl::PointNormal> p_n;
  PrecisionStopWatch t;
  std::string file_ = argv[1];
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

  Eigen::Vector3f normal;
  determinePlaneNormal(p, normal);
  std::cout << normal << std::endl;

  cob_3d_features::OrganizedNormalEstimationOMP<PointXYZ,Normal,PointLabel> ne;
  ne.setPixelSearchRadius(8,2,2);
  ne.setInputCloud(p);
  PointCloud<PointLabel>::Ptr labels(new PointCloud<PointLabel>);
  ne.setOutputLabels(labels);
  t.precisionStart();
  ne.compute(*n);
  std::cout << t.precisionStop() << "s\t for organized normal estimation" << std::endl;
  concatenateFields (*p, *n, p_n);
  io::savePCDFileASCII ("/home/goa/pcl_daten/normal_estimation/normals_organized.pcd", p_n);

  double good_thr=0.97;
  unsigned int ctr=0, nan_ctr=0;
  double d_sum=0;
  for(unsigned int i=0; i<p->size(); i++)
  {
    if ( pcl_isnan(n->points[i].normal[0]) ) {nan_ctr++;continue;}
    double d = normal.dot(n->points[i].getNormalVector3fMap());
    d_sum += fabs(1-fabs(d));
    if(fabs(d)>good_thr) ctr++;
  }
  std::cout << "Average error: " << d_sum/p->size() << std::endl;
  std::cout << "Ratio of good normals: " << (double)ctr/p->size() << std::endl;
  std::cout << "Invalid normals: " << nan_ctr << std::endl;

  IntegralImageNormalEstimation<PointXYZ,Normal> ne2;
  ne2.setNormalEstimationMethod (ne2.COVARIANCE_MATRIX);
  ne2.setMaxDepthChangeFactor(0.02f);
  ne2.setNormalSmoothingSize(10.0f);
  ne2.setDepthDependentSmoothing(true);
  ne2.setInputCloud(p);
  t.precisionStart();
  ne2.compute(*n);
  std::cout << t.precisionStop() << "s\t for integral image normal estimation" << std::endl;
  concatenateFields (*p, *n, p_n);
  io::savePCDFileASCII ("/home/goa/pcl_daten/normal_estimation/normals_integral.pcd", p_n);

  ctr=0;
  nan_ctr=0;
  d_sum=0;
  for(unsigned int i=0; i<p->size(); i++)
  {
    if ( pcl_isnan(n->points[i].normal[0]) ) {nan_ctr++;continue;}
    double d = normal.dot(n->points[i].getNormalVector3fMap());
    d_sum += fabs(1-fabs(d));
    if(fabs(d)>good_thr) ctr++;
  }
  std::cout << "Average error: " << d_sum/p->size() << std::endl;
  std::cout << "Ratio of good normals: " << (double)ctr/p->size() << std::endl;
  std::cout << "Invalid normals: " << nan_ctr << std::endl;

  NormalEstimationOMP<PointXYZ, Normal> ne3;
  ne3.setInputCloud(p);
  ne3.setNumberOfThreads(4);
  ne3.setKSearch(256);
  //ne3.setRadiusSearch(0.01);
  t.precisionStart();
  ne3.compute(*n);
  std::cout << t.precisionStop() << "s\t for vanilla normal estimation" << std::endl;
  concatenateFields (*p, *n, p_n);
  io::savePCDFileASCII ("/home/goa/pcl_daten/normal_estimation/normals_vanilla.pcd", p_n);

  ctr=0;
  nan_ctr=0;
  d_sum=0;
  for(unsigned int i=0; i<p->size(); i++)
  {
    if ( pcl_isnan(n->points[i].normal[0]) ) {nan_ctr++;continue;}
    double d = normal.dot(n->points[i].getNormalVector3fMap());
    d_sum += fabs(1-fabs(d));
    if(fabs(d)>good_thr) ctr++;
  }
  std::cout << "Average error: " << d_sum/p->size() << std::endl;
  std::cout << "Ratio of good normals: " << (double)ctr/p->size() << std::endl;
  std::cout << "Invalid normals: " << nan_ctr << std::endl;

  return 0;
}
