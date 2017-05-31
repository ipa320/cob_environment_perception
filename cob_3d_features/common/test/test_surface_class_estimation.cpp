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
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
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

// OpenCV:
#include <opencv2/opencv.hpp>

// Boost:
#include <boost/program_options.hpp>
#include <boost/timer.hpp>

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
/*#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>*/

// Package Includes:
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_features/fast_edge_estimation_3d.h"
#include "cob_3d_features/organized_normal_estimation.h"
#include "cob_3d_features/organized_curvature_estimation.h"
#include "cob_3d_features/curvature_classifier.h"
#include "cob_3d_features/impl/curvature_classifier.hpp"
//#include "cob_3d_mapping_features/segmentation.h"


using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string file_;
float cmax_;
int radius_;
int circle_;
float th_;
int threads_;
int rfp_;
float lower_, upper_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_), "input pcd file")
    ("cmax,t", value<float>(&cmax_)->default_value(0.5), "max curvature")
    ("threshold_modifier,m", value<float>(&th_)->default_value(4.0), "thresholdmodifier")
    ("radius,r", value<int>(&radius_)->default_value(5), "radius")
    ("circle,c", value<int>(&circle_)->default_value(2),"circle steps")
    ("feature,f", value<int>(&rfp_)->default_value(20), "set 3d edge estimation radius")
    ("lower,l", value<float>(&lower_)->default_value(1.5), "lower curvature threshold")
    ("upper,u", value<float>(&upper_)->default_value(6.0), "upper curvature threshold")
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

void applyColor(int i, PointCloud<PointLabel>::Ptr p, PointCloud<PointXYZRGB>::Ptr col)
{
  switch (p->points[i].label)
  {
  case I_SPHERE: //sphere
    col->points[i].r = 255;
    col->points[i].g = 0;
    col->points[i].b = 255;
    break;
  case I_CYL: //cylinder
    col->points[i].r = 255;
    col->points[i].g = 255;
    col->points[i].b = 0;
    break;
  case I_PLANE: //plane
    col->points[i].r = 0;
    col->points[i].g = 0;
    col->points[i].b = 255;
    break;
  case I_EDGE: //edge
    col->points[i].r = 0;
    col->points[i].g = 255;
    col->points[i].b = 0;
    break;
  case I_BORDER: //border
    col->points[i].r = 255;
    col->points[i].g = 0;
    col->points[i].b = 0;
    break;
  default:
    col->points[i].r = 100;
    col->points[i].g = 100;
    col->points[i].b = 100;
    break;
  }
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);
  //boost::timer t;
  PrecisionStopWatch t;

  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p2(new PointCloud<PointXYZRGB>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);
  PointCloud<PointLabel>::Ptr le(new PointCloud<PointLabel>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<InterestPoint>::Ptr ip3d(new PointCloud<InterestPoint>);

  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);
  *p2 = *p;

/*
  t.precisionStart();
  IntegralImageNormalEstimation<PointXYZRGB,Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setDepthDependentSmoothing(true);
  ne.setInputCloud(p);
  ne.compute(*n3);
  cout << t.precisionStop() << "s\t for IntegralImage Estimation" << endl;
*/
/*
  t.precisionStart();
  KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setRadiusSearch(0.04);
  ne.setSearchMethod(tree);
  ne.setInputCloud(p);
  ne.compute(*n);
  cout << t.precisionStop() << "s\t for Normal Estimation" << endl;
*/

  t.precisionStart();
  cob_3d_features::OrganizedNormalEstimation<PointXYZRGB, Normal, PointLabel>one;
  one.setInputCloud(p);
  one.setOutputLabels(l);
  one.setPixelSearchRadius(radius_,1,circle_); //radius,pixel,circle
  one.setSkipDistantPointThreshold(th_);
  one.compute(*n);
  cout << t.precisionStop() << "s\t for Organized Normal Estimation" << endl;

  t.precisionStart();
  cob_3d_features::OrganizedCurvatureEstimation<PointXYZRGB,Normal,PointLabel,PrincipalCurvatures>oce;
  oce.setInputCloud(p);
  oce.setInputNormals(n);
  oce.setOutputLabels(l);
  oce.setPixelSearchRadius(radius_,1,circle_);
  oce.setSkipDistantPointThreshold(th_);
  oce.setEdgeClassThreshold(cmax_);
  oce.compute(*pc);
  cout << t.precisionStop() << "s\t for Organized Curvature Estimation" << endl;

  cob_3d_features::CurvatureClassifier<PrincipalCurvatures, PointLabel>cc;
  cc.setInputCloud(pc);
  cc.setUpperThreshold(upper_);
  cc.setLowerThreshold(lower_);
  cc.setMaxMinRatio(7.0);
  cc.classifyForSegmentation(*l);

  cout << "Colorize min max values" <<endl;
  float c_max = 0;
  float c_min = 100000;

/*
  for (size_t i = 0; i < p->points.size(); i++)
  {
    c_max = std::max(n->points[i].curvature,c_max);
    c_min = std::min(n->points[i].curvature,c_min);
  }

  for (size_t i = 0; i < p->points.size(); i++)
  {
    int col = (n->points[i].curvature) / (cmax_) * 255;
    if (col > 255) col = 255;
    if (col < 0) col = 0;
    p->points[i].r = col;
    p->points[i].g = col;
    p->points[i].b = col;
  }
*/
  for (size_t i = 0; i < l->points.size(); i++)
  {
    applyColor(i, l, p);
  }

  visualization::PCLVisualizer v;
  ColorHdlRGB col_hdl(p);
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> col_hdl_single (p, 255,0,0);
//  ColorHdlRGB col_hdl2(p2);

  /* --- Viewports: ---
   *  1y
   *    | 1 | 3 |
   * .5 ----+----
   *    | 2 | 4 |
   *  0    .5    1x
   * 1:
   */
  // xmin, ymin, xmax, ymax
  int v1(0);
  v.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  v.setBackgroundColor(0,127,127, v1);
  v.addPointCloud<PointXYZRGB>(p, col_hdl, "segmented1", v1);
  //v.addPointCloudNormals<PointXYZRGB, Normal>(p,n,10,0.04,"normals1", v1);

  int v2(0);
  v.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  v.setBackgroundColor(0,127,127, v2);

  v.addPointCloud<PointXYZRGB>(p, col_hdl_single, "segmented2", v2);
  v.addPointCloudNormals<PointXYZRGB, Normal>(p,n,3,0.04,"normals2", v2);

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  /*
  cv::Mat segmented;
  vector<PointIndices> clusters;
  cob_3d_mapping_features::Segmentation seg;
  seg.propagateWavefront2(l);
  seg.getClusterIndices(l, clusters, segmented);
  cv::imshow("segmented", segmented);
  cv::waitKey();
  */

  return(0);

}
