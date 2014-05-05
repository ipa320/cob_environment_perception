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

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_features/organized_normal_estimation.h"
#include "cob_3d_features/organized_curvature_estimation.h"
#include "cob_3d_features/organized_normal_estimation_omp.h"
#include "cob_3d_features/organized_curvature_estimation_omp.h"

using namespace pcl;

int main(int argc, char** argv)
{
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);

  PrecisionStopWatch t;
  std::string file_ = argv[1];
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

  t.precisionStart();
  cob_3d_features::OrganizedNormalEstimationOMP<PointXYZRGB, Normal, PointLabel>one;
  one.setInputCloud(p);
  one.setOutputLabels(l);
  one.setPixelSearchRadius(8,1,2); //radius,pixel,circle
  one.setSkipDistantPointThreshold(12);
  one.compute(*n);
  std::cout << t.precisionStop() << "s\t for organized curvature estimation" << std::endl;

  t.precisionStart();
  cob_3d_features::OrganizedCurvatureEstimationOMP<PointXYZRGB,Normal,PointLabel,PrincipalCurvatures>oce;
  oce.setInputCloud(p);
  oce.setInputNormals(n);
  oce.setOutputLabels(l);
  oce.setPixelSearchRadius(8,1,2);
  oce.setSkipDistantPointThreshold(12);
  oce.setEdgeClassThreshold(6);
  oce.compute(*pc);
  std::cout << t.precisionStop() << "s\t for organized curvature estimation" << std::endl;


  return 0;
}
