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
 *  ROS package name: cob_3d_features
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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
#include <boost/make_shared.hpp>

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Package Includes:
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_features/organized_normal_estimation.h"
#include "cob_3d_features/organized_curvature_estimation_omp.h"
#include "cob_3d_features/curvature_classifier.h"
#include "cob_3d_features/impl/curvature_classifier.hpp"


using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string file_in_, file_out_, file_cluster_;
float rn_;
int rfp_;
float ex_th_;
bool en_one_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_in_), "input pcd file")
    ("cl,c", value<string>(&file_cluster_), "input indices file")
    ("out,o", value<string>(&file_out_), "output pcd file")
    ("normal,n", value<float>(&rn_)->default_value(0.025),
     "set normal estimation radius")
    ("feature,f", value<int>(&rfp_)->default_value(20),
     "set 3d edge estimation radius")
    ("extraction_th,x", value<float>(&ex_th_)->default_value(0.1),
      "set the strength threshold for edge extraction")
    ("one", "use organized normal estimation (one) instead of original normal estimation (ne)")
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
  en_one_ = vm.count("one");
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  boost::timer t;

  // 3D point clouds
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);

  PCDReader r;
  if (r.read(file_in_, *p) == -1) return(0);

  vector<PointIndices> indices;
  ifstream fs;

  string line;
  fs.open(file_cluster_.c_str());
  if (fs.is_open())
  {
    while(fs.good())
    {
      getline (fs,line);
      istringstream iss(line);
      PointIndices pi;
      do
      {
        int temp;
        iss >> temp;
        pi.indices.push_back(temp);
      } while(iss);
      if(pi.indices.size()>0)
        indices.push_back(pi);
    }
  }
  std::cout << "indices file read successfully" << std::endl;

  // --- Normal Estimation ---
  if (en_one_)
  {
    t.restart();
    cob_3d_features::OrganizedNormalEstimation<PointXYZRGB, Normal, PointLabel>one;
    one.setInputCloud(p);
    one.setOutputLabels(l);
    //one.setPixelSearchRadius(pns_n_,points_,circle_); //radius,pixel,circle
    one.setPixelSearchRadius(8,2,2);
    one.setSkipDistantPointThreshold(12);
    one.compute(*n);
    cout << t.elapsed() << "s\t for Organized Normal Estimation" << endl;
  }
  else
  {
    t.restart();
    #ifdef PCL_VERSION_COMPARE //fuerte
      pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
    #else //electric
      pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
    #endif
    NormalEstimation<PointXYZRGB, Normal> ne;
    ne.setRadiusSearch(rn_);
    ne.setSearchMethod(tree);
    ne.setInputCloud(p);
    ne.compute(*n);
    cout << t.elapsed() << "s\t for normal estimation" << endl;
  }
  cob_3d_features::OrganizedCurvatureEstimationOMP<PointXYZRGB, Normal, PointLabel, PrincipalCurvatures> oce;
  oce.setInputCloud(p);
  oce.setInputNormals(n);
  oce.setPixelSearchRadius(8,2,2);
  oce.setSkipDistantPointThreshold(12);

  //KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
  //ne.setRadiusSearch(rn_);
  //ne.setSearchMethod(tree);
  if (indices.size() == 0)
  {
    oce.setOutputLabels(l);
    oce.compute(*pc);
    cob_3d_features::CurvatureClassifier<PrincipalCurvatures, PointLabel>cc;
    cc.setInputCloud(pc);
    cc.classify(*l);
  }
  else
  {
    for(unsigned int i=0; i<indices.size(); i++)
    {
      std::cout << "cluster " << i << " has " << indices[i].indices.size() << " points" << std::endl;
      if(i==3)
      {
        /*for(unsigned int j=0; j<indices[i].indices.size(); j++)
          std::cout << indices[i].indices[j] << ",";
          std::cout << std::endl;*/
        //cout << i << ": " << indices[i].indices.front() << "!" << endl;
        t.restart();
        boost::shared_ptr<PointIndices> ind_ptr = boost::make_shared<PointIndices>(indices[i]);
        std::cout << ind_ptr->indices.size() << std::endl;
        oce.setIndices(ind_ptr);
        oce.setOutputLabels(l);
        oce.compute(*pc);
        cout << t.elapsed() << "s\t for principal curvature estimation" << endl;

        cob_3d_features::CurvatureClassifier<PrincipalCurvatures, PointLabel>cc;
        cc.setInputCloud(pc);
        cc.setIndices(ind_ptr);
        cc.classify(*l);
      }
    }
  }
  // colorize edges of 3d point cloud
  for (size_t i = 0; i < l->points.size(); i++)
  {
    //std::cout << l->points[i].label << std::endl;
    if (l->points[i].label == I_UNDEF)
    {
      p->points[i].r = 0;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    else if (l->points[i].label == I_NAN)
    {
      p->points[i].r = 0;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }
    else if (l->points[i].label == I_EDGE)
    {
      p->points[i].r = 0;
      p->points[i].g = 0;
      p->points[i].b = 255;
    }
    else if (l->points[i].label == I_BORDER)
    {
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    else if (l->points[i].label == I_PLANE)
    {
      p->points[i].r = 255;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }
    else if (l->points[i].label == I_CYL)
    {
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 255;
    }
    else
    {
      p->points[i].r = 255;
      p->points[i].g = 255;
      p->points[i].b = 255;
    }
  }

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
