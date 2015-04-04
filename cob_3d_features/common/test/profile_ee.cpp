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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/common/eigen.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_features/edge_estimation_3d.h"

using namespace pcl;
typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

int main(int argc, char** argv)
{
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  PrecisionStopWatch t;
  std::string file_ = "/home/goa-sf/pcd_data/pc_0.pcd";
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

  IntegralImageNormalEstimation<PointXYZRGB,Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setDepthDependentSmoothing(true);
  ne.setInputCloud(p);
  ne.compute(*n);

  t.precisionStart();
  #ifdef PCL_VERSION_COMPARE //fuerte
    search::OrganizedNeighbor<PointXYZRGB>::Ptr oTree (new search::OrganizedNeighbor<PointXYZRGB> );
  #else //electric
    OrganizedDataIndex<PointXYZRGB>::Ptr oTree (new OrganizedDataIndex<PointXYZRGB> );
  #endif
  cob_3d_features::EdgeEstimation3D<PointXYZRGB, Normal, InterestPoint> ee;
  ee.setRadiusSearch(0.04);
  ee.setSearchMethod(oTree);
  ee.setInputCloud(p);
  ee.setInputNormals(n);
  ee.dist_threshold_ = 0.02;
  ee.compute(*ip);
  std::cout << t.precisionStop() << "s\t for 3D edge estimation" << std::endl;

  for (size_t i = 0; i < ip->points.size(); i++)
  {
    int col = (ip->points[i].strength) * 255;
    if (col > 255)
    {
      p->points[i].r = 0;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }
    else if (col < 0)
    {
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    else
    {
      p->points[i].r = col;
      p->points[i].g = col;
      p->points[i].b = col;
    }
  }

  visualization::PCLVisualizer v("slow");
  ColorHdlRGB col_hdl(p);
  v.setBackgroundColor(0,127,127);
  v.addPointCloud<PointXYZRGB>(p,col_hdl, "segmented1");

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  return 0;
}
