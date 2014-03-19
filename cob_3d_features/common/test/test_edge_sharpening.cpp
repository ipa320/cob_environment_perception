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
 * \date Date of creation: 11/2012
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

#include <boost/program_options.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_features/organized_normal_estimation.h"


std::string file_in_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input pcd file")
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal NormalT;

typedef pcl::PointCloud<PointT> PCloud;
typedef pcl::PointCloud<NormalT> NCloud;

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PCloud::Ptr p(new PCloud);
  NCloud::Ptr n1(new NCloud);
  NCloud::Ptr n2(new NCloud);
  NCloud::Ptr n3(new NCloud);
  pcl::PointCloud<PointLabel>::Ptr l(new pcl::PointCloud<PointLabel>);

  pcl::PCDReader r;
  r.read(file_in_, *p);
  cob_3d_features::OrganizedNormalEstimation<PointT, NormalT, PointLabel> one;
  one.setPixelSearchRadius(8,2,2);
  one.setSkipDistantPointThreshold(8);
  one.setOutputLabels(l);
  one.setInputCloud(p);
  one.compute(*n1);

  n2->resize(n1->size());
  n3->resize(n1->size());
  int w = n3->width = n2->width = n1->width;
  int h = n3->height = n2->height = n1->height;
  *n2 = *n1;

/*
  float kernel[] = { -1.0f, -1.0f, -1.0f, -1.0f, 4.0f };
  int mask[] = { -w, 1, w, -1, 0 };
  int m_size = 5;

  for(int y=1; y<h-1; ++y) {
    for(int x=1; x<w-1; ++x) {
      int idx = y*w + x;
      for(int c=0; c<3; ++c) {
        (*n2)[ idx ].normal[c] = 0;
        for(int k=0; k<m_size; ++k) {
          if( (*n1)[ idx + mask[k] ].normal[c] != (*n1)[ idx + mask[k] ].normal[c] )
          {
            (*n2)[ idx ].normal[c] = std::numeric_limits<float>::quiet_NaN();
            break;
          }
          (*n2)[ idx ].normal[c] += (*n1)[ idx + mask[k] ].normal[c] * kernel[k];
        }
      }
      if( (*n2)[ idx ].normal[0] == (*n2)[ idx ].normal[0] )
      {
        (*n2)[ idx ].getNormalVector3fMap() = (*n2)[ idx ].getNormalVector3fMap().normalized();
      }
    }
  }
*/
  int mask_x[] = { -1, 0, 1 };
  int mask_y[] = { -w, 0, -w };
  float kernel[] = { 0.25f, 0.5f, 0.25f };
  //float kernel[] = { 0.006f, 0.061f, 0.242f, 0.383f, 0.242f, 0.061f, 0.006f };
  int kernel_size = 3;

  for(int i=1; i<h*w; ++i) {
    memset(&((*n2)[i].normal), 0, 3*sizeof(float));

    for(int k=0; k<kernel_size; ++k) {
      if( (*n1)[ i + mask_x[k] ].normal[2] != (*n1)[ i + mask_x[k] ].normal[2] ) {
        (*n2)[i].getNormalVector3fMap() = (*n1)[i].getNormalVector3fMap();
        break;
      }
      (*n2)[i].getNormalVector3fMap() += kernel[k] * (*n1)[ i + mask_x[k] ].getNormalVector3fMap();
    }
  }

  for(int i=0; i<w; ++i)
    (*n3)[i].normal[0] = (*n3)[i].normal[1] = (*n3)[i].normal[2] = std::numeric_limits<float>::quiet_NaN();
  for(int i=h*(w-1); i<h*w; ++i)
    (*n3)[i].normal[0] = (*n3)[i].normal[1] = (*n3)[i].normal[2] = std::numeric_limits<float>::quiet_NaN();

  for(int i=w; i<h*(w-1); ++i) {
    memset(&((*n3)[i].normal), 0, 3*sizeof(float));

    for(int k=0; k<kernel_size; ++k) {
      if( (*n2)[ i + mask_y[k] ].normal[2] != (*n2)[ i + mask_y[k] ].normal[2] ) {
        (*n3)[i].getNormalVector3fMap() = (*n2)[i].getNormalVector3fMap();
        break;
      }
      (*n3)[i].getNormalVector3fMap() += kernel[k] * (*n2)[ i + mask_y[k] ].getNormalVector3fMap();
    }

    if( (*n3)[i].normal[2] == (*n3)[i].normal[2] )
      (*n3)[i].getNormalVector3fMap() = (*n3)[i].getNormalVector3fMap().normalized();
  }

  for(int i=0; i<h*w; ++i) {
    if( (*n3)[i].normal[2] != (*n3)[i].normal[2] ) continue;
    Eigen::Vector3f n_org = (*n1)[i].getNormalVector3fMap();
    Eigen::Vector3f n_dif = n_org - (*n3)[i].getNormalVector3fMap();
    if(n_dif(0) * n_dif(0) + n_dif(1) * n_dif(1) + n_dif(2) * n_dif(2) > 0.001f)
      (*n2)[i].getNormalVector3fMap() = (n_org + n_dif).normalized();
    else
      (*n2)[i].getNormalVector3fMap() = (*n3)[i].getNormalVector3fMap();
  }

  visualization::PCLVisualizer v;
  visualization::PointCloudColorHandlerCustom<PointT> blue_hdl (p, 0 ,0 ,230);
  int v1(0), v2(0);
  v.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  v.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  v.addPointCloud<PointT>(p, blue_hdl, "l_points", v1);
  v.addPointCloudNormals<PointT,NormalT>(p, n3, 10, 0.05, "l_normal", v1);

  v.addPointCloud<PointT>(p, blue_hdl, "r_points", v2);
  v.addPointCloudNormals<PointT,NormalT>(p, n2, 10, 0.05, "r_normal", v2);

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

}
