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
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 01/2012
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


// Boost:
#include <boost/program_options.hpp>

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Package Includes:
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_features/organized_normal_estimation.h"
#include "cob_3d_mapping_features/organized_curvature_estimation.h"
#include "cob_3d_mapping_tools/io.h"

using namespace std;
using namespace pcl;

string file_in_;
vector<string> file_out_(2,"");
bool fixed_max_ = false, fixed_min_ = false;
float max_pc1_ = 0, min_pc1_ = 0, th_;
float max_pc2_ = 0, min_pc2_ = 0;
int radius_;
int circle_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in", value<string>(&file_in_), "input pcd file")
    ("out", value< vector<string> >(&file_out_), "output files, first ppm, [second ppm]")
    ("radius,r", value<int>(&radius_)->default_value(5), "radius")
    ("circle,c", value<int>(&circle_)->default_value(2),"circle steps")
    ("threshold_modifier,t", value<float>(&th_)->default_value(4.0), "thresholdmodifier")
    ("max_pc,M", value<float>(&max_pc1_), "maximum value for pc1")
    ("min_pc,m", value<float>(&min_pc1_), "minimum value for pc1")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out", 2);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("in") || !vm.count("out"))
  {
    cout << "Reads an raw organized point cloud and extracts \n"
	 << "the principal curvatures as ppm file.\n"
	 << "This ppm was used to label a point cloud manually" << endl;
    cout << options << endl;
    exit(0);
  }
  if (vm.count("max_pc"))
    fixed_max_ = true;

  if (vm.count("min_pc"))
    fixed_min_ = true;

}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p2(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);
  PCDReader r;
  if (r.read(file_in_, *p) == -1) return(0);
  *p2 = *p;

  PrecisionStopWatch t;
  t.precisionStart();
  cob_3d_mapping_features::OrganizedNormalEstimation<PointXYZRGB, Normal, PointLabel>one;
  one.setInputCloud(p);
  one.setOutputLabels(l);
  one.setPixelSearchRadius(radius_,1,circle_); //radius,pixel,circle
  one.setSkipDistantPointThreshold(th_);
  one.compute(*n);
  cout << t.precisionStop() << "s\t for Organized Normal Estimation" << endl;

  t.precisionStart();
  cob_3d_mapping_features::OrganizedCurvatureEstimation<PointXYZRGB,Normal,PointLabel,PrincipalCurvatures>oce;
  oce.setInputCloud(p);
  oce.setInputNormals(n);
  oce.setOutputLabels(l);
  oce.setPixelSearchRadius(radius_,1,circle_);
  oce.setSkipDistantPointThreshold(th_);
  oce.setEdgeClassThreshold(0.1);
  oce.compute(*pc);
  cout << t.precisionStop() << "s\t for Organized Curvature Estimation" << endl;

  for (size_t i = 0; i < pc->points.size(); i++)
  {
    if (pcl_isnan(pc->points[i].pc1))
      continue;
    if(!fixed_max_) 
      max_pc1_ = max (pc->points[i].pc1, max_pc1_);
    if(!fixed_min_) 
      min_pc1_ = min (pc->points[i].pc1, min_pc1_);

    max_pc2_ = max (pc->points[i].pc2, max_pc2_);
    min_pc2_ = min (pc->points[i].pc2, min_pc2_);
  }
  max_pc2_ = 0.1;
  min_pc2_ = 0.0;
  cout << "Max_pc1 = " << max_pc1_ << " | Min_pc1 = " << min_pc1_ << endl;
  cout << "Max_pc2 = " << max_pc2_ << " | Min_pc2 = " << min_pc2_ << endl;

  double grd_position;
  uint8_t rgb[3];

  for (size_t i = 0; i < pc->points.size(); i++)
  {
    if (!pcl_isnan(pc->points[i].pc1))
    {
      grd_position = (pc->points[i].pc1 - min_pc1_) / (max_pc1_ - min_pc1_);
      cob_3d_mapping_tools::getGradientColor(grd_position, rgb);
      p->points[i].r = (int)rgb[0];
      p->points[i].g = (int)rgb[1];
      p->points[i].b = (int)rgb[2];

      grd_position = (pc->points[i].pc2 - min_pc2_) / (max_pc2_ - min_pc2_);
      cob_3d_mapping_tools::getGradientColor(grd_position, rgb);
      p2->points[i].r = (int)rgb[0];
      p2->points[i].g = (int)rgb[1];
      p2->points[i].b = (int)rgb[2];
    }
    else 
    {
      p->points[i].r = 125;
      p->points[i].g = 125;
      p->points[i].b = 125;

      p2->points[i].r = 125;
      p2->points[i].g = 125;
      p2->points[i].b = 125;
    }
  }

  cob_3d_mapping_tools::PPMWriter ppmW;
  if (file_out_[0] != "")
    ppmW.writeRGB(file_out_[0], *p);

  if (file_out_[1] != "")
    ppmW.writeRGB(file_out_[1], *p2);

  return 0;
}
