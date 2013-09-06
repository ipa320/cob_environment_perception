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
 *  ROS package name: cob_3d_meshing
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 06/2013
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

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/PolygonMesh.h>

#include <cob_3d_mapping_filters/downsample_filter.h>
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_mapping_common/sensor_model.h>
#include <cob_3d_features/organized_normal_estimation_omp.h>
#include <cob_3d_segmentation/impl/fast_segmentation.hpp>

#include <cob_3d_meshing/mesh_types.h>
#include <cob_3d_meshing/mesh_conversion.h>
#include <cob_3d_meshing/mesh_decimation.h>


//== GLOBAL VARS ===============================================================
std::string file_in_;
std::string file_out_;
bool save_pcl_;
int n_vertices_;


int readOptions(int argc, char** argv)
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help,h", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input folder with data points")
    ("out,o", value<std::string>(&file_out_), "out file with data points")
    ("n_vertices,n", value<int>(&n_vertices_)->default_value(10000),
     "decimate to n vertices")
    ("pcl", "save pcl results")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out",1);
  variables_map vm;
  store(command_line_parser(argc, argv)
        .options(options).positional(p_opt).run(), vm);
  notify(vm);

  if(vm.count("help") || argc == 1)
  { std::cout << options << std::endl; return(-1); }
  if(vm.count("pcl")) save_pcl_ = true;
  else save_pcl_ = false;

  return 0;
}


int main(int argc, char** argv)
{
  if(readOptions(argc, argv)<0) return 0;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    input(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    down(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr
    normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointLabel>::Ptr
    labels(new pcl::PointCloud<PointLabel>);

  cob_3d_features::OrganizedNormalEstimationOMP
    <pcl::PointXYZRGB, pcl::Normal, PointLabel> one;

  cob_3d_segmentation::FastSegmentation
    <pcl::PointXYZRGB, pcl::Normal, PointLabel> seg;

  cob_3d_mapping_filters::DownsampleFilter
    <pcl::PointXYZRGB> dsf;

  pcl::OrganizedFastMesh
    <pcl::PointXYZRGB> ofm;

  pcl::PCDReader r;
  r.read(file_in_, *input);

  PrecisionStopWatch t, tt;

  tt.precisionStart();
  t.precisionStart();
  dsf.setInputCloud(input);
  dsf.filter(*down);
  std::cout << "DownsampleFilter took "
            << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  one.setInputCloud(down);
  one.setOutputLabels(labels);
  one.setPixelSearchRadius(8,2,2);
  one.setSkipDistantPointThreshold(8);
  one.compute(*normals);
  std::cout << "OrganizedNormalEstimationOMP took "
            << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  seg.setInputCloud(down);
  seg.setNormalCloudIn(normals);
  seg.setLabelCloudInOut(labels);
  seg.setSeedMethod(cob_3d_segmentation::SEED_RANDOM);
  seg.compute();

  std::map<int,Eigen::Vector4f> params;

  typedef typename cob_3d_segmentation::FastSegmentation
    <pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterPtr ClusterPtr;
  for (ClusterPtr c = seg.clusters()->begin(); c!=seg.clusters()->end(); ++c)
  {
    if(c->size() < 5) {continue;}
    seg.clusters()->computeClusterComponents(c);
    float d = fabs(c->getCentroid().dot(c->pca_point_comp3));
    params[c->id()] = Eigen::Vector4f(c->pca_point_comp3(0),
                                      c->pca_point_comp3(1),
                                      c->pca_point_comp3(2),
                                      d);
  }
  std::cout << "FastSegmentation took "
            << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  typedef cob_3d_meshing::MeshProperties::Normals<pcl::Normal> PropNormalT;
  typedef cob_3d_meshing::Mesh<PropNormalT> MeshT;
  PropNormalT prop_normal_hdl(normals);
  MeshT mesh( prop_normal_hdl );
  cob_3d_meshing::MeshConversion<>::fromPointCloud<pcl::PointXYZRGB,MeshT>(down,mesh);
  cob_3d_meshing::MeshDecimation<MeshT>::quadratic(&mesh, n_vertices_);
  std::cout << "MeshSimplification took "
            << t.precisionStop() << "s." << std::endl;

  std::cout << "Total Process took "
            << tt.precisionStop() << "s." << std::endl;

  pcl::PolygonMesh pcl_mesh;
  t.precisionStart();
  ofm.setInputCloud(down);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
  ofm.reconstruct(pcl_mesh);
  std::cout << "OrganizedFastMesh took "
            << t.precisionStop() << "s." << std::endl;


  try
  {
    if(save_pcl_) pcl::io::savePLYFile(file_out_, pcl_mesh);
    else
    {
      if ( !mesh.savePLY(file_out_) )
      {
        std::cerr << "Cannot write mesh to file " << file_out_ << std::endl;
        return 1;
      }
    }
  }
  catch( std::exception& x )
  {
    std::cerr << x.what() << std::endl;
    return 1;
  }

  return 0;
}
