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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 02/2011
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>


#include <cob_3d_mapping_common/label_defines.h>
#include "cob_3d_mapping_features/plane_extraction.h"

std::string file_in_;
std::string label_out_;
bool borders_;

typedef pcl::PointXYZRGB PointT;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input pcd file")
    ("label_out,l", value<std::string>(&label_out_), "save labeled file to")
    ("borders,b", "enable border visuaization")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || argc == 1)
  {
    std::cout << options << std::endl;
    exit(0);
  }
  if (vm.count("borders")) borders_=true;
  else borders_=false;
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  pcl::PointCloud<PointT>::Ptr p(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr p_vox(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr p_out(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr p_hull(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile<PointT> (file_in_, *p);
  std::cout << "loaded " << p->size() << " Points..." << std::endl;

  *p_out = *p;

  pcl::VoxelGrid<PointT> voxel;
  voxel.setSaveLeafLayout(true);
  voxel.setInputCloud(p);
  voxel.setLeafSize(0.03,0.03,0.03);
  voxel.filter(*p_vox);

  for(pcl::PointCloud<PointT>::iterator it=p_vox->begin(); it!=p_vox->end(); ++it) { it->rgba = LBL_SPH; }

  std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> > > v_cloud_hull;
  std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
  std::vector<pcl::ModelCoefficients> v_coefficients_plane;
  PlaneExtraction pe;
  pe.setSaveToFile(false);
  pe.setClusterTolerance(0.06);
  pe.setMinPlaneSize(150);
  pe.setAlpha(0.2);
  pe.extractPlanes(p_vox, v_cloud_hull, v_hull_polygons, v_coefficients_plane);

  // colorize voxel cloud using extracted indices
  const float rand_max_inv = 1.0f/ RAND_MAX;
  for (size_t pl=0; pl<pe.extracted_planes_indices_.size(); ++pl)
  {
    int r = (float)rand() * rand_max_inv * 255;
    int g = (float)rand() * rand_max_inv * 255;
    int b = (float)rand() * rand_max_inv * 255;
    int color = ( r << 16 | g << 8 | b );
    for(std::vector<int>::iterator idx=pe.extracted_planes_indices_[pl].begin(); idx!=pe.extracted_planes_indices_[pl].end(); ++idx)
    {
      p_vox->points[*idx].rgba = color;
    }
  }

  // extract color from voxelized cloud
  for (pcl::PointCloud<PointT>::iterator it=p_out->begin(); it!=p_out->end(); ++it)
  {
    if (it->z != it->z) { it->rgba = LBL_UNDEF; }
    else { it->rgba = p_vox->points[voxel.getCentroidIndex(*it)].rgba; }
  }

  if (borders_)
  {
    // colorize borders
    int mask[] = { -p_out->width, 1, p_out->width, -1 };
    int x, y, count, curr_color;
    pcl::PointCloud<PointT>::Ptr p_copy(new pcl::PointCloud<PointT>);
    *p_copy = *p_out;
    for (int idx=0; idx<p_out->size(); ++idx)
    {
      curr_color = (*p_copy)[idx].rgba;
      x = idx % p_out->width; y = idx / p_out->width;
      if (x==0 || y==0 || x == p_out->width-1 || y == p_out->height-1) { (*p_out)[idx].rgba = LBL_BORDER; continue; }
      count = 0;
      for(int i=0;i<4;++i) { if (curr_color != (*p_copy)[idx+mask[i]].rgba) { ++count; } }
      if (count > 3 || count < 1)
      {
        if ((*p_copy)[idx].rgba != LBL_SPH) { (*p_out)[idx].rgba = LBL_PLANE; }
        continue;
      }
      (*p_out)[idx].rgba = LBL_BORDER;
    }
  }

  for (size_t pl=0; pl<v_cloud_hull.size(); ++pl)
  {
    int r = (float)rand() * rand_max_inv * 255;
    int g = (float)rand() * rand_max_inv * 255;
    int b = (float)rand() * rand_max_inv * 255;
    int color = ( r << 16 | g << 8 | b );
    //std::cout << "Size hull: " << v_cloud_hull[pl].size() << std::endl;
    for (size_t p_idx=0; p_idx<v_cloud_hull[pl].size(); ++p_idx)
    {
      p_hull->push_back(v_cloud_hull[pl].points[p_idx]);
      p_hull->back().rgba = color;
    }
  }

  p_hull->width = p_hull->points.size();
  p_hull->height = 1;


  if (label_out_ != "")
  {
    pcl::io::savePCDFileASCII<PointT>(label_out_, *p_out);
    return 0;
  }
  pcl::visualization::PCLVisualizer v;
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> chdl_p(p_out);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> chdl_hull(p_hull);

  /* --- Viewports: ---
   *  1y
   *    | 1 | 2 |
   * .5 ----+----
   *    | 3 | 4 |
   *  0    .5    1x
   * 1:
   */
  // xmin, ymin, xmax, ymax

  int v1(0);
  v.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  v.addPointCloud<PointT>(p_out, chdl_p, "planes", v1);

  int v2(0);
  v.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  v.addPointCloud<PointT>(p_hull, chdl_hull, "hull", v2);

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  return 0;
}


