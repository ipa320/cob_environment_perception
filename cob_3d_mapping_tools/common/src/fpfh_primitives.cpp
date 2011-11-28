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
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
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

/*!
 * @brief Generates FPFH of a single point on various shapes
 * @todo For some shape sizes this point not always lies in the center of the shape
 */

#include "cob_3d_mapping_tools/point_generator.h"
#include <cob_3d_mapping_common/label_defines.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <sys/time.h>
#include <sys/resource.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace pcl;

typedef Eigen::Vector3f Vec;
typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> ColorHdlRGBA;

string folder_;
bool vis_;
float step_;
float noise_;
float rn_;
float rf_;
float r_;


struct ft_config
{
  float s;
  float rng;
  float rn;
  float rf;
  string file_name;
};

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  
  options_description cmd_line("Options");
  cmd_line.add_options()
    ("help,h", "produce help messsage")
    ("out,o", value<string>(&folder_), "output folder")
    ("step,s", value<float>(&step_)->default_value(0.005f), "define point density")
    ("noise,g", value<float>(&noise_)->default_value(0.0f), "add gaussian noise")
    ("normal,n", value<float>(&rn_)->default_value(0.015f), 
     "neighborood radius normal estimation")
    ("feature,f", value<float>(&rf_)->default_value(0.025f), 
     "neighborood radius feature estimation")
    ("shaperadius,r", value<float>(&r_)->default_value(0.05f), 
     "radius for size of cylinder and spheres")
    ("vis,v", value<bool>(&vis_)->default_value(false), "enable visualization")
    ;

  variables_map vm;
  store(command_line_parser(argc, argv)
	.options(cmd_line).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("in"))
  {
    cout << "Generates FPFH of a single point on various shapes" << endl;
    cout << "TODO: For some shape sizes the point not always lies in the center of the shape" << endl;
    cout << cmd_line << endl;
    exit(0);
  }
}

// convert a float to string
string fl2label(const float & f, const size_t & precision)
{
  if (f == 0) 
    return string(precision, '0');

  stringstream ss;
  ss << f;
  string s = ss.str();
  s = s.substr(2,s.length());
  if (precision > s.length())
    s += string(precision - s.length(), '0');
  return (s);
}

void generateName(ft_config * cfg, string shape)
{
  string name = fl2label(cfg->rf, 3) + "fpfh_" 
    + fl2label(cfg->rn, 3) + "n_" 
    + fl2label(cfg->rng, 4) + "rng_"
    + shape + ".pcd";
  cfg->file_name = name;
}

void generateFeature(ft_config * cfg, PointIndices::Ptr idx, PointCloud<PointXYZRGBA>::Ptr & p_in,
		     PointCloud<Normal>::Ptr & normal_out)
{
  PointCloud<FPFHSignature33>::Ptr fpfh (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr qpoint (new PointCloud<FPFHSignature33>);
  PointIndices::Ptr pi(new PointIndices);
  vector<float> d;
  KdTree<PointXYZRGBA>::Ptr tree(new KdTreeFLANN<PointXYZRGBA>());
  tree->setInputCloud(p_in);
  tree->radiusSearch(idx->indices[0], cfg->rf, pi->indices, d);

  for (size_t i = 0; i < pi->indices.size(); ++i)
    p_in->points[pi->indices[i]].rgba = 0x0000ff;
  p_in->points[idx->indices[0]].rgba = LBL_PLANE;

  NormalEstimation<PointXYZRGBA, Normal> norm;
  norm.setInputCloud(p_in);
  norm.setSearchMethod(tree);
  norm.setRadiusSearch(cfg->rn);
  norm.compute(*normal_out);

  FPFHEstimation<PointXYZRGBA, Normal, FPFHSignature33> fpfhE;
  fpfhE.setInputCloud(p_in);
  fpfhE.setInputNormals(normal_out);
  fpfhE.setSearchMethod(tree);
  fpfhE.setRadiusSearch(cfg->rf);
  fpfhE.compute(*fpfh);

  copyPointCloud<FPFHSignature33>(*fpfh, *idx, *qpoint);
  if (!folder_.empty()) io::savePCDFileASCII<FPFHSignature33>(folder_ + cfg->file_name, *qpoint);
}

void plane (ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		 PointCloud<Normal>::Ptr & normal_out)
{
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  pg.generatePlane(cfg->s, 0.25f * Vec::UnitZ(), 0.2f * Vec::UnitX(), 0.2f * Vec::UnitY());
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(out->points.size() * 0.5 + 20);
  generateName(cfg, "plane_");
  generateFeature(cfg, idx,  out, normal_out);
}

void edge_convex(ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		 PointCloud<Normal>::Ptr & normal_out)
{
  Vec o(0.0, 0.0, 0.25);
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  pg.generateEdge(cfg->s, o, 0.5f * Vec::UnitX(), Vec(0.0f, 1.0f, 1.0f).normalized() * 0.2f);
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(out->points.size() * 0.5 - 1);
  generateName(cfg, "edge_convex_");
  generateFeature(cfg, idx,  out, normal_out);
}

void edge_concave(ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		  PointCloud<Normal>::Ptr & normal_out)
{
  Vec o(0.0, 0.0, 0.25);
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  pg.generateEdge(cfg->s, o, -0.5f * Vec::UnitX(), Vec(0.0f, 1.0f, -1.0f).normalized() * 0.2f);
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(out->points.size() * 0.5 - 1);
  generateName(cfg, "edge_concave_");
  generateFeature(cfg, idx, out, normal_out);
}

void cylinder_concave(ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		      PointCloud<Normal>::Ptr & normal_out, float r)
{
  Vec o(0.0, 0.0, 0.25);
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  pg.generateCylinder(cfg->s, o, 0.2f * Vec::UnitX(), r * Vec::UnitY(), M_PI);
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(out->points.size() * 0.5 - 1);
  generateName(cfg, "cylinder_concave_");
  generateFeature(cfg, idx, out, normal_out);
}

void cylinder_convex(ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		     PointCloud<Normal>::Ptr & normal_out, float r)
{
  Vec o(0.0, 0.0, 2.0);
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  pg.generateCylinder(cfg->s, o, 0.2f * Vec::UnitX(), -r * Vec::UnitY(), M_PI);
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(out->points.size() * 0.5 - 1);
  generateName(cfg, "cylinder_convex_");
  generateFeature(cfg, idx, out, normal_out);
}

void sphere_convex(ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		   PointCloud<Normal>::Ptr & normal_out, float r)
{
  Vec o(0.0, 0.0, 0.25);
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  float step = cfg->s;
  pg.generateSphere(step, o, -Vec::UnitZ(), r, 0.45 * M_PI);
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(1);
  generateName(cfg, "sphere_convex_");
  generateFeature(cfg, idx, out, normal_out);
}

void sphere_concave(ft_config * cfg, PointCloud<PointXYZRGBA>::Ptr & out, 
		   PointCloud<Normal>::Ptr & normal_out, float r)
{
  Vec o(0.0, 0.0, 0.25);
  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  pg.setGaussianNoise(cfg->rng);
  pg.setOutputCloud(out);
  float step = cfg->s;
  pg.generateSphere(step, o, Vec::UnitZ(), r, 0.45 * M_PI);
  PointIndices::Ptr idx(new PointIndices);
  idx->indices.push_back(1);
  generateName(cfg, "sphere_concave_");
  generateFeature(cfg, idx, out, normal_out);
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  ft_config *c1;
  c1 = new ft_config;
  c1->s = step_;
  c1->rng = noise_;
  c1->rn = rn_;
  c1->rf = rf_;

  PointCloud<PointXYZRGBA>::Ptr p (new PointCloud<PointXYZRGBA>);
  PointCloud<Normal>::Ptr n (new PointCloud<Normal>);
  PointCloud<PointXYZRGBA>::Ptr p2 (new PointCloud<PointXYZRGBA>);
  PointCloud<Normal>::Ptr n2 (new PointCloud<Normal>);
  PointCloud<PointXYZRGBA>::Ptr p3 (new PointCloud<PointXYZRGBA>);
  PointCloud<Normal>::Ptr n3 (new PointCloud<Normal>);

  plane(c1, p, n);

  p.reset(new PointCloud<PointXYZRGBA>);
  n.reset(new PointCloud<Normal>);
  edge_convex(c1, p, n);

  p.reset(new PointCloud<PointXYZRGBA>);
  n.reset(new PointCloud<Normal>);
  edge_concave(c1, p, n);

  p3.reset(new PointCloud<PointXYZRGBA>);
  n3.reset(new PointCloud<Normal>);
  cylinder_convex(c1, p3, n3, r_);

  p.reset(new PointCloud<PointXYZRGBA>);
  n.reset(new PointCloud<Normal>);
  cylinder_concave(c1, p, n, r_);

  p.reset(new PointCloud<PointXYZRGBA>);
  n.reset(new PointCloud<Normal>);
  sphere_convex(c1, p, n, r_);

  p.reset(new PointCloud<PointXYZRGBA>);
  n.reset(new PointCloud<Normal>);
  sphere_concave(c1, p, n, r_);

  visualization::PCLVisualizer vis;
  int vp(0);
  vis.setBackgroundColor(0.7, 0.7, 0.7, vp);
  vis.addCoordinateSystem(0.1,vp);
  ColorHdlRGBA hdl(p3);
  vis.addPointCloud<PointXYZRGBA>(p3, hdl, "points", vp);
  vis.addPointCloudNormals<PointXYZRGBA, Normal>(p3, n3, 4, 0.02, "normals", vp);


  while(!vis.wasStopped())
  {
    vis.spinOnce(100);
    usleep(100000);
  }

}
