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
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 02/2012
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

#include <cstdlib>

#include <cob_3d_mapping_common/label_defines.h>
#include <cob_3d_evaluation_features/label_results.h>

#include <boost/program_options.hpp>
#include <fstream>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/rsd.h>
#include <pcl/features/fpfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

// The following global variables hold the boost::program_option option:
float pass_depth_;

bool rsd_enable_;
bool rsd_mls_enable_;
bool rsd_vox_enable_;
float rsd_vox_;
float rsd_rn_;
float rsd_rf_;

float r_limit_;
float r_low;
float r_high;
float r_r1;
float r_r2;

bool pc_enable_;
bool pc_mls_enable_;
bool pc_vox_enable_;
float pc_vox_;
float pc_rn_;
float pc_rf_;

float c_low;
float c_high;
float c_r1; // sphere / cyl
float c_r2; // corner / edge

bool fpfh_enable_;
bool fpfh_mls_enable_;
bool fpfh_vox_enable_;
float fpfh_vox_;
float fpfh_rn_;
float fpfh_rf_;

string file_in_;
string fpfh_svm_model_;
string camera_pos_;
bool quiet_;

string fl2label(const float & f, const size_t & precision=3)
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

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;

  string config_file;
  options_description cmd("Command line options");
  options_description cfg("Config file options");

  cmd.add_options()
    ("help", "produce help message")
    ("config,c", value<string>(&config_file)->default_value("config/accuracy_evaluator.cfg"),
     "name of a file for configuration.")
    ("in,i", value<string>(&file_in_), "labeled input pcd file")
    ("svm_model,s", value<string>(&fpfh_svm_model_), "set svm to use svm")
    ("camera,C", value<string>(&camera_pos_), "specify a file for initial camera position")
    ("quiet,Q", "disable visualization")
    ;
  cfg.add_options()
    ("pass.depth", value<float>(&pass_depth_), "set depth of passthrough filter")

    ("rsd.enabled", value<bool>(&rsd_enable_), "enable rsd estimation")
    ("rsd.enable_mls", value<bool>(&rsd_mls_enable_), "enable surface smoothing for rsd")
    ("rsd.enable_vox", value<bool>(&rsd_vox_enable_), "enable voxelgrid filtering for rsd")
    ("rsd.voxel_size", value<float>(&rsd_vox_), "set a voxelgrid size for rsd")
    ("rsd.limit_rmax", value<float>(&r_limit_), "set the radius limit for rsd feature values")
    ("rsd.rn", value<float>(&rsd_rn_), "set the normal estimation radius for rsd")
    ("rsd.rf", value<float>(&rsd_rf_), "feature estimation radius for rsd")
    ("rsd.r_low", value<float>(&r_low), "")
    ("rsd.r_high", value<float>(&r_high), "")
    ("rsd.r_r1", value<float>(&r_r1), "sphere / cylinder")
    ("rsd.r_r2", value<float>(&r_r2), "edge / corner")

    ("pc.enabled", value<bool>(&pc_enable_), "enable principal curvature estimation")
    ("pc.enable_mls", value<bool>(&pc_mls_enable_), "enable surface smoothing for pc")
    ("pc.enable_vox", value<bool>(&pc_vox_enable_), "enable voxelgrid filtering for pc")
    ("pc.voxel_size", value<float>(&pc_vox_), "set a voxelgrid size for pc")
    ("pc.rn", value<float>(&pc_rn_), "set the normal estimation radius for pc")
    ("pc.rf", value<float>(&pc_rf_), "feature estimation radius for pc")
    ("pc.c_low",  value<float>(&c_low), "")
    ("pc.c_high", value<float>(&c_high), "")
    ("pc.c_r1",   value<float>(&c_r1), "sphere / cylinder")
    ("pc.c_r2",   value<float>(&c_r2), "edge / corner")

    ("fpfh.enabled", value<bool>(&fpfh_enable_), "enable fpfh estimation")
    ("fpfh.enable_mls", value<bool>(&fpfh_mls_enable_), "enable surface smoothing for fpfh")
    ("fpfh.enable_vox", value<bool>(&fpfh_vox_enable_), "enable voxelgrid filtering for fpfh")
    ("fpfh.voxel_size", value<float>(&fpfh_vox_), "set a voxelgrid size for fpfh")
    ("fpfh.rn", value<float>(&fpfh_rn_), "set the normal estimation radius for fpfh")
    ("fpfh.rf", value<float>(&fpfh_rf_), "feature estimation radius for fpfh")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1);
  cmd.add(cfg);

  variables_map vm;
  store(command_line_parser(argc, argv)
	.options(cmd)
	.positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << cmd << endl;
    exit(0);
  }
  if (vm.count("quiet")) quiet_ = true;
  else quiet_ = false;

  ifstream ifs(config_file.c_str());
  if (!ifs)
    cout << "cannot open config file: " << config_file << endl;
  else
  {
    store(parse_config_file(ifs, cmd), vm);
    notify(vm);
  }
}

/*! @brief runs the whole processing pipeline for FPFH features
 *
 * @note At the moment the evaluation results will be printed to console.
 *
 * @param[in] in the labeled input point cloud
 * @param[out] ref_out the reference point cloud after the preprocessing steps
 * @param[out] fpfh_out the labeled point cloud after the classifing process
 */
void processFPFH(const PointCloud<PointXYZRGB>::Ptr in,
                 PointCloud<PointXYZRGB>::Ptr ref_out,
                 PointCloud<PointXYZRGB>::Ptr fpfh_out)
{
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>());
  PointCloud<FPFHSignature33>::Ptr fpfh(new PointCloud<FPFHSignature33>());

  // Passthrough filtering (needs to be done to remove NaNs)
  cout << "FPFH: Pass (with " << in->points.size() << " points)" << endl;
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0f, pass_depth_);
  pass.filter(*ref_out);

  // Optional voxelgrid filtering
  if (fpfh_vox_enable_)
  {
    cout << "FPFH: Voxel (with " << ref_out->points.size() << " points)" << endl;
    VoxelGrid<PointXYZRGB> vox;
    vox.setInputCloud(ref_out);
    vox.setLeafSize(fpfh_vox_, fpfh_vox_, fpfh_vox_);
    vox.filter(*ref_out);
  }

  #ifdef PCL_VERSION_COMPARE //fuerte
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
  #else //electric
    pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
  #endif
  //KdTree<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>());
  tree->setInputCloud(ref_out);

  // Optional surface smoothing
  if(fpfh_mls_enable_)
  {
    cout << "FPFH: MLS (with " << ref_out->points.size() << " points)" << endl;

    #ifdef PCL_VERSION_COMPARE
      std::cerr << "MLS has changed completely in PCL 1.7! Requires redesign of entire program" << std::endl;
      exit(0);
    #else
      MovingLeastSquares<PointXYZRGB, Normal> mls;
      mls.setInputCloud(ref_out);
      mls.setOutputNormals(n);
      mls.setPolynomialFit(true);
      mls.setPolynomialOrder(2);
      mls.setSearchMethod(tree);
      mls.setSearchRadius(fpfh_rn_);
      mls.reconstruct(*ref_out);
    #endif
    cout << "FPFH: flip normals (with " << ref_out->points.size() << " points)" << endl;
    for (size_t i = 0; i < ref_out->points.size(); ++i)
    {
      flipNormalTowardsViewpoint(ref_out->points[i], 0.0f, 0.0f, 0.0f,
                                 n->points[i].normal[0],
                                 n->points[i].normal[1],
                                 n->points[i].normal[2]);
    }
  }
  else
  {
    cout << "FPFH: Normals (with " << ref_out->points.size() << " points)" << endl;
    NormalEstimation<PointXYZRGB, Normal> norm;
    norm.setInputCloud(ref_out);
    norm.setSearchMethod(tree);
    norm.setRadiusSearch(fpfh_rn_);
    norm.compute(*n);
  }

  // FPFH estimation
  #ifdef PCL_VERSION_COMPARE //fuerte
    tree.reset(new pcl::search::KdTree<PointXYZRGB>());
  #else //electric
    tree.reset(new KdTreeFLANN<PointXYZRGB> ());
  #endif
  tree->setInputCloud(ref_out);
  cout << "FPFH: estimation (with " << ref_out->points.size() << " points)" << endl;
  FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfhE;
  fpfhE.setInputCloud(ref_out);
  fpfhE.setInputNormals(n);
  fpfhE.setSearchMethod(tree);
  fpfhE.setRadiusSearch(fpfh_rf_);
  fpfhE.compute(*fpfh);

  cout << "FPFH: classification " << endl;
  *fpfh_out = *ref_out;

#if CV_MAJOR_VERSION == 2
  CvSVM svm;
  svm.load(fpfh_svm_model_.c_str());
#else
  cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load(fpfh_svm_model_.c_str());
#endif
  cv::Mat fpfh_histo(1, 33, CV_32FC1);

  int exp_rgb, pre_rgb, predict;
  cob_3d_mapping_common::LabelResults stats(fl2label(fpfh_rn_),fl2label(fpfh_rf_),fpfh_mls_enable_);
  for (size_t idx = 0; idx < ref_out->points.size(); idx++)
  {
    exp_rgb = *reinterpret_cast<int*>(&ref_out->points[idx].rgb); // expected label
    memcpy(fpfh_histo.ptr<float>(0), fpfh->points[idx].histogram, sizeof(fpfh->points[idx].histogram));
#if CV_MAJOR_VERSION == 2
    predict = (int)svm.predict(fpfh_histo);
#else
    predict = (int)svm->predict(fpfh_histo);
#endif
    //cout << predict << endl;
    switch(predict)
    {
    case SVM_PLANE:
      pre_rgb = LBL_PLANE;
      if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_PLANE]++;
      break;
    case SVM_EDGE:
      pre_rgb = LBL_EDGE;
      if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGE]++;
      if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGECORNER]++;
      break;
    case SVM_COR:
      pre_rgb = LBL_COR;
      if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[EVAL_COR]++;
      if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGECORNER]++;
      break;
    case SVM_SPH:
      pre_rgb = LBL_SPH;
      if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[EVAL_SPH]++;
      if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CURVED]++;
      break;
    case SVM_CYL:
      pre_rgb = LBL_CYL;
      if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CYL]++;
      if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CURVED]++;
      break;
    default:
      pre_rgb = LBL_UNDEF;
      break;
    }

    switch(exp_rgb)
    {
    case LBL_PLANE:
      if (pre_rgb != exp_rgb) stats.fn[EVAL_PLANE]++;
      stats.exp[EVAL_PLANE]++;
      break;
    case LBL_EDGE:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_EDGE]++;
	if (pre_rgb != LBL_COR) stats.fn[EVAL_EDGECORNER]++;
      }
      stats.exp[EVAL_EDGE]++;
      stats.exp[EVAL_EDGECORNER]++;
      break;
    case LBL_COR:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_COR]++;
	if (pre_rgb != LBL_EDGE) stats.fn[EVAL_EDGECORNER]++;
      }
      stats.exp[EVAL_COR]++;
      stats.exp[EVAL_EDGECORNER]++;
      break;
    case LBL_SPH:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_SPH]++;
	if (pre_rgb != LBL_CYL) stats.fn[EVAL_CURVED]++;
      }
      stats.exp[EVAL_SPH]++;
      stats.exp[EVAL_CURVED]++;
      break;
    case LBL_CYL:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_CYL]++;
	if (pre_rgb != LBL_SPH) stats.fn[EVAL_CURVED]++;
      }
      stats.exp[EVAL_CYL]++;
      stats.exp[EVAL_CURVED]++;
      break;
    default:
      stats.undef++;
      break;
    }
    fpfh_out->points[idx].rgb = *reinterpret_cast<float*>(&pre_rgb);
  }
  cout << "FPFH:\n" << stats << endl << endl;
}

void processPC(const PointCloud<PointXYZRGB>::Ptr in,
	       PointCloud<PointXYZRGB>::Ptr ref_out,
	       PointCloud<PointXYZRGB>::Ptr pc_out)
{
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>());
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>());

  // passthrough filtering (needed to remove NaNs)
  cout << "PC: Pass (with " << in->points.size() << " points)" << endl;
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0f, pass_depth_);
  pass.filter(*ref_out);

  // Optional voxelgrid filtering
  if (pc_vox_enable_)
  {
    cout << "PC: Voxel (with " << ref_out->points.size() << " points)" << endl;
    VoxelGrid<PointXYZRGB> vox;
    vox.setInputCloud(ref_out);
    vox.setLeafSize(pc_vox_, pc_vox_, pc_vox_);
    vox.filter(*ref_out);
  }

  #ifdef PCL_VERSION_COMPARE //fuerte
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
  #else //electric
    KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
  #endif
  tree->setInputCloud(ref_out);

  // Optional surface smoothing
  if(pc_mls_enable_)
  {
    cout << "PC: MLS (with " << ref_out->points.size() << " points)" << endl;

    #ifdef PCL_VERSION_COMPARE
      std::cerr << "MLS has changed completely in PCL 1.7! Requires redesign of entire program" << std::endl;
      exit(0);
    #else
      MovingLeastSquares<PointXYZRGB, Normal> mls;
      mls.setInputCloud(ref_out);
      mls.setOutputNormals(n);
      mls.setPolynomialFit(true);
      mls.setPolynomialOrder(2);
      mls.setSearchMethod(tree);
      mls.setSearchRadius(pc_rn_);
      mls.reconstruct(*ref_out);
    #endif
    cout << "PC: flip normals (with " << ref_out->points.size() << " points)" << endl;
    for (size_t i = 0; i < ref_out->points.size(); ++i)
    {
      flipNormalTowardsViewpoint(ref_out->points[i], 0.0f, 0.0f, 0.0f,
				 n->points[i].normal[0],
				 n->points[i].normal[1],
				 n->points[i].normal[2]);
    }
  }
  else
  {
    cout << "PC: Normals (with " << ref_out->points.size() << " points)" << endl;
    NormalEstimation<PointXYZRGB, Normal> norm;
    norm.setInputCloud(ref_out);
    norm.setSearchMethod(tree);
    norm.setRadiusSearch(pc_rn_);
    norm.compute(*n);
  }

  // estimate PC
  #ifdef PCL_VERSION_COMPARE //fuerte
    tree.reset(new pcl::search::KdTree<PointXYZRGB>());
  #else //electric
    tree.reset(new KdTreeFLANN<PointXYZRGB> ());
  #endif
  tree->setInputCloud(ref_out);
  cout << "PC: estimation (with " << ref_out->points.size() << " points)" << endl;
  PrincipalCurvaturesEstimation<PointXYZRGB, Normal, PrincipalCurvatures> pcE;
  pcE.setInputCloud(ref_out);
  pcE.setInputNormals(n);
  pcE.setSearchMethod(tree);
  pcE.setRadiusSearch(pc_rf_);
  pcE.compute(*pc);

  cout << "PC: classification " << endl;
  *pc_out = *ref_out;

  int exp_rgb, pre_rgb;
  float c_max,c_min;
  cob_3d_mapping_common::LabelResults stats(fl2label(pc_rn_),fl2label(pc_rf_),pc_mls_enable_);

  // apply rules to PC results
  for (size_t idx = 0; idx < ref_out->points.size(); idx++)
  {
    exp_rgb = *reinterpret_cast<int*>(&ref_out->points[idx].rgb); // expected label
    c_max = pc->points[idx].pc1;
    c_min = pc->points[idx].pc2;

    if ( c_max < c_low )
    {
      pre_rgb = LBL_PLANE;
      if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_PLANE]++;
    }
    else if (c_max > c_high)
    {
      if (c_max < c_r2 * c_min)
      {
	pre_rgb = LBL_COR;
	if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[EVAL_COR]++;
      }
      else
      {
	pre_rgb = LBL_EDGE;
	if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGE]++;
      }
      // special case:  combined class for corner and edge
      if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF)
	stats.fp[EVAL_EDGECORNER]++;
    }
    else
    {
      if (c_max < c_r1 * c_min)
      {
	pre_rgb = LBL_SPH;
	if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[EVAL_SPH]++;
      }
      else
      {
	pre_rgb = LBL_CYL;
	if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CYL]++;
      }
      // special case:  combined class for sphere and cylinder
      if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF)
	stats.fp[EVAL_CURVED]++;
    }

    switch(exp_rgb)
    {
    case LBL_PLANE:
      if (pre_rgb != exp_rgb) stats.fn[EVAL_PLANE]++;
      stats.exp[EVAL_PLANE]++;
      break;
    case LBL_EDGE:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_EDGE]++;
	if (pre_rgb != LBL_COR) stats.fn[EVAL_EDGECORNER]++;
      }
      stats.exp[EVAL_EDGE]++;
      stats.exp[EVAL_EDGECORNER]++;
      break;
    case LBL_COR:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_COR]++;
	if (pre_rgb != LBL_EDGE) stats.fn[EVAL_EDGECORNER]++;
      }
      stats.exp[EVAL_COR]++;
      stats.exp[EVAL_EDGECORNER]++;
      break;
    case LBL_SPH:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_SPH]++;
	if (pre_rgb != LBL_CYL) stats.fn[EVAL_CURVED]++;
      }
      stats.exp[EVAL_SPH]++;
      stats.exp[EVAL_CURVED]++;
      break;
    case LBL_CYL:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_CYL]++;
	if (pre_rgb != LBL_SPH) stats.fn[EVAL_CURVED]++;
      }
      stats.exp[EVAL_CYL]++;
      stats.exp[EVAL_CURVED]++;
      break;
    default:
      stats.undef++;
      break;
    }
    pc_out->points[idx].rgb = *reinterpret_cast<float*>(&pre_rgb);
  }
  cout << "PC:\n" << stats << endl << endl;
}

void processRSD(const PointCloud<PointXYZRGB>::Ptr in,
		PointCloud<PointXYZRGB>::Ptr ref_out,
		PointCloud<PointXYZRGB>::Ptr rsd_out)
{
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>());
  PointCloud<PrincipalRadiiRSD>::Ptr rsd(new PointCloud<PrincipalRadiiRSD>());

  // passthrough filtering (needed to remove NaNs)
  cout << "RSD: Pass (with " << in->points.size() << " points)" << endl;
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0f, pass_depth_);
  pass.filter(*ref_out);

  // optional voxelgrid filtering
  if (rsd_vox_enable_)
  {
    cout << "RSD: Voxel (with " << ref_out->points.size() << " points)" << endl;
    VoxelGrid<PointXYZRGB> vox;
    vox.setInputCloud(ref_out);
    vox.setLeafSize(rsd_vox_, rsd_vox_, rsd_vox_);
    vox.filter(*ref_out);
  }

  #ifdef PCL_VERSION_COMPARE //fuerte
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
  #else //electric
    KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
  #endif
  tree->setInputCloud(ref_out);

  // optional surface smoothing
  if(rsd_mls_enable_)
  {
    cout << "RSD: MLS (with " << ref_out->points.size() << " points)" << endl;

    #ifdef PCL_VERSION_COMPARE
      std::cerr << "MLS has changed completely in PCL 1.7! Requires redesign of entire program" << std::endl;
      exit(0);
    #else
      MovingLeastSquares<PointXYZRGB, Normal> mls;
      mls.setInputCloud(ref_out);
      mls.setOutputNormals(n);
      mls.setPolynomialFit(true);
      mls.setPolynomialOrder(2);
      mls.setSearchMethod(tree);
      mls.setSearchRadius(rsd_rn_);
      mls.reconstruct(*ref_out);
    #endif

    cout << "RSD: flip normals (with " << ref_out->points.size() << " points)" << endl;
    for (size_t i = 0; i < ref_out->points.size(); ++i)
    {
      flipNormalTowardsViewpoint(ref_out->points[i], 0.0f, 0.0f, 0.0f,
				 n->points[i].normal[0],
				 n->points[i].normal[1],
				 n->points[i].normal[2]);
    }
  }
  else
  {
    cout << "RSD: Normals (with " << ref_out->points.size() << " points)" << endl;
    NormalEstimation<PointXYZRGB, Normal> norm;
    norm.setInputCloud(ref_out);
    norm.setSearchMethod(tree);
    norm.setRadiusSearch(rsd_rn_);
    norm.compute(*n);
  }

  tree->setInputCloud(ref_out);

  // RSD estimation
  cout << "RSD: estimation (with " << ref_out->points.size() << " points)" << endl;
  RSDEstimation<PointXYZRGB, Normal, PrincipalRadiiRSD> rsdE;
  rsdE.setInputCloud(ref_out);
  rsdE.setInputNormals(n);
  rsdE.setSearchMethod(tree);
  rsdE.setPlaneRadius(r_limit_);
  rsdE.setRadiusSearch(rsd_rf_);
  rsdE.compute(*rsd);

  cout << "RSD: classification " << endl;
  *rsd_out = *ref_out;

  // apply RSD rules for classification
  int exp_rgb, pre_rgb;
  float r_max,r_min;
  cob_3d_mapping_common::LabelResults stats(fl2label(rsd_rn_),fl2label(rsd_rf_),rsd_mls_enable_);
  for (size_t idx = 0; idx < ref_out->points.size(); idx++)
  {
    exp_rgb = *reinterpret_cast<int*>(&ref_out->points[idx].rgb); // expected label
    r_max = rsd->points[idx].r_max;
    r_min = rsd->points[idx].r_min;

    if ( r_min > r_high )
    {
      pre_rgb = LBL_PLANE;
      if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_PLANE]++;
    }
    else if (r_min < r_low)
    {
      if (r_max < r_r2 * r_min)
      {
	pre_rgb = LBL_COR;
	if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[EVAL_COR]++;
      }
      else
      {
	pre_rgb = LBL_EDGE;
	if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGE]++;
      }
      // special case:  combined class for corner and edge
      if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF)
	stats.fp[EVAL_EDGECORNER]++;
    }
    else
    {
      if (r_max < r_r1 * r_min)
      {
	pre_rgb = LBL_SPH;
	if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[EVAL_SPH]++;
      }
      else
      {
	pre_rgb = LBL_CYL;
	if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CYL]++;
      }
      // special case:  combined class for sphere and cylinder
      if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF)
	stats.fp[EVAL_CURVED]++;
    }

    switch(exp_rgb)
    {
    case LBL_PLANE:
      if (pre_rgb != exp_rgb) stats.fn[EVAL_PLANE]++;
      stats.exp[EVAL_PLANE]++;
      break;
    case LBL_EDGE:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_EDGE]++;
	if (pre_rgb != LBL_COR) stats.fn[EVAL_EDGECORNER]++;
      }
      stats.exp[EVAL_EDGE]++;
      stats.exp[EVAL_EDGECORNER]++;
      break;
    case LBL_COR:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_COR]++;
	if (pre_rgb != LBL_EDGE) stats.fn[EVAL_EDGECORNER]++;
      }
      stats.exp[EVAL_COR]++;
      stats.exp[EVAL_EDGECORNER]++;
      break;
    case LBL_SPH:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_SPH]++;
	if (pre_rgb != LBL_CYL) stats.fn[EVAL_CURVED]++;
      }
      stats.exp[EVAL_SPH]++;
      stats.exp[EVAL_CURVED]++;
      break;
    case LBL_CYL:
      if (pre_rgb != exp_rgb)
      {
	stats.fn[EVAL_CYL]++;
	if (pre_rgb != LBL_SPH) stats.fn[EVAL_CURVED]++;
      }
      stats.exp[EVAL_CYL]++;
      stats.exp[EVAL_CURVED]++;
      break;
    default:
      stats.undef++;
      break;
    }
    rsd_out->points[idx].rgb = *reinterpret_cast<float*>(&pre_rgb);
  }
  cout << "RSD:\n" << stats << endl << endl;
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGB>::Ptr p_raw(new PointCloud<PointXYZRGB>());
  PointCloud<PointXYZRGB>::Ptr p_rsd_out(new PointCloud<PointXYZRGB>());
  PointCloud<PointXYZRGB>::Ptr p_rsd_ref(new PointCloud<PointXYZRGB>());
  PointCloud<PointXYZRGB>::Ptr p_pc_out(new PointCloud<PointXYZRGB>());
  PointCloud<PointXYZRGB>::Ptr p_pc_ref(new PointCloud<PointXYZRGB>());
  PointCloud<PointXYZRGB>::Ptr p_fpfh_out(new PointCloud<PointXYZRGB>());
  PointCloud<PointXYZRGB>::Ptr p_fpfh_ref(new PointCloud<PointXYZRGB>());

  PCDReader r;
  if(r.read(file_in_, *p_raw) == -1)
    cout << "Could not read file " << file_in_ << endl;
  cout << "Read pcd file \"" << file_in_ << "\" (Points: " << p_raw->points.size() << ", width: "
       << p_raw->width << ", height: " << p_raw->height << ")" << endl;

  cout << "Headline: \n\n"<< cob_3d_mapping_common::writeHeader() << endl << endl;

  if (rsd_enable_)
    processRSD(p_raw, p_rsd_ref, p_rsd_out);
  if (pc_enable_)
    processPC(p_raw, p_pc_ref, p_pc_out);
  if (fpfh_enable_)
    processFPFH(p_raw, p_fpfh_ref, p_fpfh_out);

  if (quiet_) return (0);

  boost::shared_ptr<visualization::PCLVisualizer> v;
  if (camera_pos_ != "")
  {
    int argc_dummy = 3;
    char *argv_dummy[] = {(char*)argv[0], "-cam", (char*)camera_pos_.c_str(), NULL};
    v.reset(new visualization::PCLVisualizer(argc_dummy, argv_dummy, file_in_));
  }
  else
  {
    v.reset(new visualization::PCLVisualizer(file_in_));
  }



  /* --- Viewports: ---
   *  1y
   *    | 1 | 3 |
   * .5 ----+----
   *    | 2 | 4 |
   *  0    .5    1x
   * 1:
   */
  int v1(0);
  ColorHdlRGB col_hdl1(p_rsd_ref);
  v->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
  v->setBackgroundColor(255,255,255, v1);
  v->addPointCloud<PointXYZRGB>(p_rsd_ref, col_hdl1, "raw", v1);

  // 2:
  int v2(0);
  ColorHdlRGB col_hdl2(p_fpfh_out);
  v->createViewPort(0.0, 0.0, 0.5, 0.5, v2);
  v->setBackgroundColor(255,255,255, v2);
  v->addPointCloud<PointXYZRGB>(p_fpfh_out, col_hdl2, "fpfh", v2);

  // 3:
  int v3(0);
  ColorHdlRGB col_hdl3(p_rsd_out);
  v->createViewPort(0.5, 0.5, 1.0, 1.0, v3);
  v->setBackgroundColor(255,255,255, v3);
  v->addPointCloud<PointXYZRGB>(p_rsd_out, col_hdl3, "rsd", v3);

  // 4:
  int v4(0);
  ColorHdlRGB col_hdl4(p_pc_out);
  v->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
  v->setBackgroundColor(255,255,255, v4);
  v->addPointCloud<PointXYZRGB>(p_pc_out, col_hdl4, "pc", v4);


  while(!v->wasStopped())
  {
    v->spinOnce(100);
    usleep(100000);
  }
  return(0);
}
