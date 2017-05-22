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
// OpenCV:
#include <opencv2/opencv.hpp>

#include <boost/program_options.hpp>
// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Packages Includes:
#include "cob_3d_features/edge_estimation_3d.h"
#include "cob_3d_features/edge_estimation_2d.h"
#include "cob_3d_mapping_common/point_types.h"

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string file_points;
float normal_radius;
float edge_th;
bool rgbEdges, depthEdges;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_points), "input pcd file")
    ("normal_radius,R", value<float>(&normal_radius)->default_value(0.01),
     "radius normal estimation")
    ("edgeth,e",value<float>(&edge_th)->default_value(0.5), "threshold edge")
    ("rgbedges,c", value<bool>(&rgbEdges)->default_value(false), "calculate color edges")
    ("depthedges,d", value<bool>(&depthEdges)->default_value(false), "calculate range image edges")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  PointCloud<InterestPoint>::Ptr ip2(new PointCloud<InterestPoint>);

  PCDReader r;
  if(r.read(file_points, *p) == -1) return(0);

  #ifdef PCL_VERSION_COMPARE //fuerte
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
  #else //electric
    pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
  #endif
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setRadiusSearch(normal_radius);
  ne.setSearchMethod(tree);
  ne.setInputCloud(p);
  ne.compute(*n);

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
  cout << "Done!" << endl;

  if (rgbEdges)
  {
    cout << "Start 2D part" << endl;
    cv::Mat sobel, laplace, combined, color;
    cob_3d_features::EdgeEstimation2D<PointXYZRGB, InterestPoint> ee2;
    ee2.setInputCloud(p);
    ee2.getColorImage(color);
    ee2.computeEdges(*ip2);
    ee2.computeEdges(sobel, laplace, combined);
    cv::imshow("Color", color);
    cv::imshow("Combined", combined);
    cv::imshow("Laplace", laplace);
    cv::imshow("Sobel", sobel);
    cv::waitKey();
  }

  if (depthEdges)
  {
    cout << "Start depth part" << endl;
    cv::Mat sobel, laplace, combined, range;
    cob_3d_features::EdgeEstimation2D<PointXYZRGB, InterestPoint> ee3;
    ee3.setInputCloud(p);
    ee3.getRangeImage(range, 0.0, 0.0);
    ee3.computeEdgesFromRange(sobel, laplace, combined);
    cv::imshow("Range", range);
    cv::imshow("Combined", combined);
    cv::imshow("Laplace", laplace);
    cv::imshow("Sobel", sobel);
    cv::waitKey();
  }

  for (size_t i = 0; i < ip2->points.size(); i++)
  {
    /*int color = max(0.0f, min(ip2->points[i].strength*255, 255.0f) );
    p->points[i].r = color;
    p->points[i].g = color;
    p->points[i].b = color;*/

    if (ip->points[i].strength >= 0.3 && ip->points[i].strength <1.0)
    {
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    /*else if (ip->points[i].strength > edge_th)
    {
      p->points[i].r = 0;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }*/
    else
    {
      p->points[i].r = 0;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
  }

  boost::shared_ptr<visualization::PCLVisualizer> v;
  v.reset(new visualization::PCLVisualizer(file_points));
  v->setBackgroundColor(0,127,127);
  ColorHdlRGB col_hdl1(p);
  v->addPointCloud<PointXYZRGB>(p, col_hdl1, "edge");

  while(!v->wasStopped())
  {
    v->spinOnce(100);
    usleep(100000);
  }
  return(0);
}
