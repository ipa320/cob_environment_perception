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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 10/2011
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

// OpenCV:
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <highgui.h>

// Boost:
#include <boost/program_options.hpp>
#include <boost/timer.hpp>

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Package Includes:
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/fast_edge_estimation_3d_omp.h"
#include "cob_3d_mapping_features/edge_estimation_3d.h"
#include "cob_3d_mapping_features/edge_estimation_2d.h"
#include "cob_3d_mapping_features/edge_extraction.h"
#include "cob_3d_mapping_features/segmentation.h"

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string file_in_, file_out_;
float rn_;
int rfp_;
float ex_th_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_in_), "input pcd file")
    ("out,o", value<string>(&file_out_), "output pcd file")
    ("normal,n", value<float>(&rn_)->default_value(0.025),
     "set normal estimation radius")
    ("feature,f", value<int>(&rfp_)->default_value(20),
     "set 3d edge estimation radius")
    ("extraction_th,x", value<float>(&ex_th_)->default_value(0.1),
      "set the strength threshold for edge extraction")
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
  boost::timer t;

  // 3D point clouds
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip3d(new PointCloud<InterestPoint>);
  PointCloud<InterestPoint>::Ptr ip2d(new PointCloud<InterestPoint>);
  PointCloud<PointLabel>::Ptr le(new PointCloud<PointLabel>);
  PointCloud<PointLabel>::Ptr lc(new PointCloud<PointLabel>);

  // 2D representation
  cv::Mat color, sobel, laplace, edge_3d, combined_2d, combined_3d, segmented;

  vector<PointIndices> clusters;

  PCDReader r;
  if (r.read(file_in_, *p) == -1) return(0);

  t.restart();
  KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setRadiusSearch(rn_);
  ne.setSearchMethod(tree);
  ne.setInputCloud(p);
  ne.compute(*n);
  cout << t.elapsed() << "s\t for normal estimation" << endl;

  t.restart();
  cob_3d_mapping_features::FastEdgeEstimation3DOMP<PointXYZRGB, Normal, InterestPoint> ee3d;
  ee3d.setPixelSearchRadius(rfp_,1,1);
  ee3d.setInputCloud(p);
  ee3d.setInputNormals(n);
  ee3d.compute(*ip3d);
  cout << t.elapsed() << "s\t for 3D edge estimation" << endl;

  t.restart();
  cob_3d_mapping_features::EdgeEstimation2D<PointXYZRGB, InterestPoint> ee2d;
  ee2d.setInputCloud(p);
  ee2d.getColorImage(color);
  ee2d.computeEdges(*ip2d);
  ee2d.computeEdges(sobel, laplace, combined_2d);
  cout << t.elapsed() << "s\t for 2D edge estimation" << endl;

  t.restart();
  cob_3d_mapping_features::EdgeExtraction<InterestPoint,PointLabel> eex;
  eex.setInput2DEdges(ip2d);
  eex.setInput3DEdges(ip3d);
  eex.setThreshold(ex_th_);
  eex.extractEdges(*le);
  cout << t.elapsed() << "s\t for edge extraction" << endl;

  t.restart();
  cob_3d_mapping_features::Segmentation seg;
  seg.propagateWavefront2(le);
  seg.getClusterIndices(le, clusters, segmented);
  cout << t.elapsed() << "s\t for clustering" << endl;
  ofstream fs;
  fs.open (file_out_.c_str());
  for(unsigned int i=0; i<clusters.size(); i++)
  {
    for(unsigned int j=0; j<clusters[i].indices.size(); j++)
    {
      fs << clusters[i].indices[j] << " ";
    }
    fs << "\n";
  }
  fs.close();

  // map 3d edges on 2d image:
  edge_3d = cv::Mat(ip3d->height, ip3d->width, CV_32FC1);
  for (unsigned int i=0; i < ip3d->height; i++)
  {
    for (unsigned int j=0; j < ip3d->width; j++)
    {
      edge_3d.at<float>(i,j) = ip3d->points[i*ip3d->width+j].strength;
    }
  }

  // colorize edges of 3d point cloud
  for (size_t i = 0; i < le->points.size(); i++)
  {
    if (le->points[i].label == 0)
    {
      p->points[i].r = 255;
      p->points[i].g = 255;
      p->points[i].b = 255;
    }
    else if (le->points[i].label == 1)
    {
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    else
    {
      p->points[i].r = 0;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }
  }

  cv::imshow("Color", color);
  cv::imshow("sobel", sobel);
  cv::imshow("laplace", laplace);
  cv::imshow("combined2d", combined_2d);
  cv::imshow("edge3d", edge_3d);
  cv::imshow("segmented", segmented);
  cv::waitKey();

  visualization::PCLVisualizer v;
  v.setBackgroundColor(0,127,127);
  ColorHdlRGB col_hdl(p);
  v.addPointCloud<PointXYZRGB>(p,col_hdl, "segmented");

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }
  return(0);
}
