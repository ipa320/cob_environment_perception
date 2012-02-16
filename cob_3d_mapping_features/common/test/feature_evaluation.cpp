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

/*
 * accuracy_evaluator.cpp
 *      Evaluation framework for RSD, PC and FPFH:
 *      Voxel -> Normals -> Features -> Classifier -> Groundtruth Comparer
 *
 *  Created on: 19.07.2011
 *      Author: goa-sf
 */

#include <cob_3d_mapping_common/label_defines.h>
#include <cob_3d_mapping_features/feature_evaluation.h>
#include <cob_3d_mapping_features/knn_classifier.h>

#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include <fstream>
#include <math.h>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <opencv2/ml/ml.hpp>

using namespace std;
using namespace pcl;
using namespace cob_3d_mapping_features;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> ColorHdlRGBA;

string file_in_;
string folder_out_;
string fpfh_svm_model_;
string fpfh_knn_values_;
string fpfh_knn_labels_;
string log_prefix_;
string camera_pos_;

float nan2zero(float f)
{
  return (isnan(f) ? 100.0 : f);
}

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

void EvalResults::updateValues()
{
  if (tp_[0] != -1 && tn_[0] != -1)
    return;

  all_ = 0;
  for(size_t i = 0; i < NUM_LBL; ++i)
  {
    all_ += ref_[i];
  }
  for(size_t i = 0; i < NUM_LBL; ++i)
  {
    tp_[i] = ref_[i] - fn_[i];
    tn_[i] = all_ - ref_[i] - fp_[i];
  }
}

float EvalResults::calcFalsePosAcc(const int &label)
{
  if (label > NUM_LBL)
    return(-1);

  return (nan2zero((float)fp_[label] * 100.0f / ((float)all_ - (float)ref_[label])));
}

float EvalResults::calcFalseNegAcc(const int &label)
{
  if (label > NUM_LBL)
    return(-1);

  return (nan2zero((float)fn_[label] * 100.0f / (float)ref_[label]));
}

float EvalResults::calcTruePosAcc(const int &label)
{
  if (label > NUM_LBL)
    return(-1);

  updateValues();
  return (nan2zero((float)tp_[label] * 100.0f / (float)ref_[label]));
}

float EvalResults::calcTrueNegAcc(const int &label)
{
  if (label > NUM_LBL)
    return(-1);  

  updateValues();
  return (nan2zero((float)tn_[label] * 100.0f / ((float)all_ - (float)ref_[label])));
}

float EvalResults::calcProportion(const int &label)
{
  if (label > NUM_LBL)
    return(-1);  

  updateValues();
  return (nan2zero((float)ref_[label] * 100 / (float)all_));
}

float EvalResults::calcAllAcc()
{
  updateValues();
  int sum_tp = 0;
  for(size_t i = 0; i < NUM_LBL; ++i)
  {
    sum_tp += tp_[i];
  }
  return (nan2zero((float)sum_tp * 100.0f / (float)all_)); 
}

float EvalResults::calcEdgePlaneAcc()
{
  int sum_tp = tp_[I_PLANE] + tp_[I_EDGE] + getTPnEnP();
  return (nan2zero((float)sum_tp * 100.0f / (float)all_)); 
}

void EvalResults::printAllToConsole(const string &prefix)
{
  updateValues();
  cout << "\n" << prefix << "\t\tAll\t\tFN\tFP\t\tTN\tTP" << endl;
  cout << "Shape\t\tProportion\tError\t\t\tAccuracy" << endl;
  cout << "-------------------------------------------------------------------------" << endl;
  printToConsole(I_PLANE, "Plane\t");
  cout << "-------------------------------------------------------------------------" << endl;
  printToConsole(I_EDGE, "Edge\t");
  cout << "-------------------------------------------------------------------------" << endl;
  printToConsole(I_SPH, "Sphere\t");
  cout << "-------------------------------------------------------------------------" << endl;
  printToConsole(I_CYL, "Cylinder");
  cout << "-------------------------------------------------------------------------" << endl;
  cout << "Undef\t\t" << undef_ << endl;
  cout << "\n-------------------------------------------------------------------------" << endl;

  cout << "NotEdgePlane" << "\t"
       << ref_[I_SPH] + ref_[I_CYL] << "\t\t"
       << ref_[I_SPH] + ref_[I_CYL] - getTPnEnP() << "\t"
       << fn_[I_PLANE] + fn_[I_EDGE] << "\t\t" 
       << tp_[I_PLANE] + tp_[I_EDGE] << "\t" 
       << getTPnEnP() << "\t" << endl;
  streamsize prec = cout.precision(4);       
  cout << "\t\t" 
       << calcProportion(I_SPH) + calcProportion(I_CYL) << "%\t\t"
       << nan2zero((ref_[I_SPH] + ref_[I_CYL] - getTPnEnP()) * 100.0f / (ref_[I_SPH] + ref_[I_CYL])) << "%\t"
       << nan2zero((fn_[I_PLANE] + fn_[I_EDGE]) * 100.0f / (ref_[I_PLANE] + ref_[I_EDGE])) << "%\t\t"
       << nan2zero((tp_[I_PLANE] + tp_[I_EDGE]) * 100.0f / (ref_[I_PLANE] + ref_[I_EDGE])) << "%\t"
       << nan2zero(getTPnEnP() * 100.0f / (ref_[I_SPH] + ref_[I_CYL])) << "%" << endl;
  cout << "-------------------------------------------------------------------------\n" << endl;

  cout << "All\t\t" << "PointAcc:\t" << calcAllAcc() << "%\t\t"
       << "ClassAcc:\t"
       << (calcTruePosAcc(I_PLANE)+calcTruePosAcc(I_EDGE)+calcTruePosAcc(I_CYL)+calcTruePosAcc(I_SPH))/4 << "%" << endl;

  cout << "Edge/Plane\t" << "PointAcc:\t" << calcEdgePlaneAcc() << "%\t\t"
       << "ClassAcc:\t"
       << (calcTruePosAcc(I_PLANE)+calcTruePosAcc(I_EDGE)+nan2zero((getTPnEnP()*100.0f/(ref_[I_SPH]+ref_[I_CYL])))) / 3 << "%\n" << endl;

  cout.precision(prec);
}

void EvalResults::printToConsole(const int &label, const string &prefix)
{
  cout << prefix << "\t"
       << ref_[label] << "\t\t"
       << fn_[label] << "\t"
       << fp_[label] << "\t\t" 
       << tn_[label] << "\t" 
       << tp_[label] << "\t" << endl;
  streamsize prec = cout.precision(4);       
  cout << "\t\t" 
       << calcProportion(label) << "%\t\t"
       << calcFalseNegAcc(label) << "%\t"
       << calcFalsePosAcc(label) << "%\t\t"
       << calcTrueNegAcc(label) << "%\t"
       << calcTruePosAcc(label) << "%" << endl;
  cout.precision(prec);
}

string EvalResults::getLabel()
{
  stringstream ss;
  switch (alg_type_)
  {
  case RES_FPFH:
    ss << fl2label(fpfh_r_, 3) << "fpfh_" << fl2label(fpfh_normal_r_, 3) << "n_"
       << (fpfh_mls_enable_ ? 1 : 0) << "mls_" 
       << (fpfh_vox_enable_ ? fl2label(fpfh_vox_, 3) : "000") << "v";
    break;

  case RES_RSD:
    ss << fl2label(rsd_r_, 3) << "rsd_" << fl2label(rsd_normal_r_, 3) << "n_"
       << (rsd_mls_enable_ ? 1 : 0) << "mls_" 
       << (rsd_vox_enable_ ? fl2label(rsd_vox_, 3) : "000") << "v";
    break;

  case RES_PC:
    ss << fl2label(pc_r_, 3) << "pc_" << fl2label(pc_normal_r_, 3) << "n_"
       << (pc_mls_enable_ ? 1 : 0) << "mls_" 
       << (pc_vox_enable_ ? fl2label(pc_vox_, 3) : "000") << "v";
    break;

  default:
    break;
  }
  return ss.str();
}

void EvalResults::writeTimerLog(const string &file)
{
  struct tm * timeinfo;
  time_t rawtime;
  time(&rawtime);
  char * time = asctime(localtime(&rawtime));
  time[strlen(time)-1] = '\0';

  fstream f;
  f.open(file.c_str(), fstream::out | fstream::app);
  f << (string)time << ": " << getLabel() << " " << points_ << " " << t_normal_ << " " << t_feature_ << " " << t_classifier_ << endl;
  f.close();
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
    ("out,o", value<string>(&folder_out_), "output folder, set to \"\" to disable output")
    ("svm_model,s", value<string>(&fpfh_svm_model_), "set svm to use svm")
    ("knn_k,k", value<int>(&fpfh_knn_k_)->default_value(1), "set k for knn")
    ("knn_values,n", value<string>(&fpfh_knn_values_), "set knn_values and knn_labels to use KNN")
    ("knn_labels,N", value<string>(&fpfh_knn_labels_), "set knn_values and knn_labels to use KNN")
    ("log,l", value<string>(&log_prefix_), "define a logfile for time measurement")
    ("camera,C", value<string>(&camera_pos_), "specify a file for initial camera position");
  cfg.add_options()
    ("vis.enable", value<bool>(&vis_enable_), "enable visualization")
    ("pass.depth", value<float>(&pass_depth_), "set depth of passthrough filter")

    ("rsd.enabled", value<bool>(&rsd_enable_), "enable rsd estimation")
    ("rsd.enable_mls", value<bool>(&rsd_mls_enable_), "enable surface smoothing for rsd")
    ("rsd.enable_vox", value<bool>(&rsd_vox_enable_), "enable voxelgrid filtering for rsd")
    ("rsd.voxel_size", value<float>(&rsd_vox_), "set a voxelgrid size for rsd")
    ("rsd.normal_radius", value<float>(&rsd_normal_r_), "set the normal estimation radius for rsd")
    ("rsd.limit_rmax", value<float>(&rsd_r_max_), "set the radius limit for rsd feature values")
    ("rsd.radius", value<float>(&rsd_r_), "feature estimation radius for rsd")
    ("rsd.rmin_upper", value<float>(&rsd_r_min_th_upper_), "everything above is plane")
    ("rsd.rmin_cylinder_lower", value<float>(&rsd_r_min_th_cylinder_lower_), 
     "everything below is edge, should be the same value as rsd.rmin_sphere_lower")
    ("rsd.rmin_sphere_lower", value<float>(&rsd_r_min_th_sphere_lower_), 
     "everything below is edge, should be the same value as rsd.rmin_cylinder_lower")
    ("rsd.rmax_rmin_ratio", value<float>(&rsd_max_min_ratio_), 
     "ratio for rsd to differentiate between cylinder and sphere")

    ("pc.enabled", value<bool>(&pc_enable_), "enable principal curvature estimation")
    ("pc.enable_mls", value<bool>(&pc_mls_enable_), "enable surface smoothing for pc")
    ("pc.enable_vox", value<bool>(&pc_vox_enable_), "enable voxelgrid filtering for pc")
    ("pc.voxel_size", value<float>(&pc_vox_), "set a voxelgrid size for pc")
    ("pc.normal_radius", value<float>(&pc_normal_r_), "set the normal estimation radius for pc")
    ("pc.radius", value<float>(&pc_r_), "feature estimation radius for pc")
    ("pc.cmax_cylinder_upper", value<float>(&pc_c_max_th_cylinder_upper_), 
     "everything above is edge, should be the same value as pc.cmax_sphere_upper")
    ("pc.cmax_sphere_upper", value<float>(&pc_c_max_th_sphere_upper_),
     "everything above is edge, should be the same value as pc.cmax_cylinder_upper")
    ("pc.cmax_lower", value<float>(&pc_c_max_th_lower_), "everything below is plane")
    ("pc.cmax_cmin_ratio", value<float>(&pc_max_min_ratio_), 
     "ratio for pc to differentiate between cylinder and sphere")

    ("fpfh.enabled", value<bool>(&fpfh_enable_), "enable fpfh estimation")
    ("fpfh.enable_mls", value<bool>(&fpfh_mls_enable_), "enable surface smoothing for fpfh")
    ("fpfh.enable_vox", value<bool>(&fpfh_vox_enable_), "enable voxelgrid filtering for fpfh")
    ("fpfh.voxel_size", value<float>(&fpfh_vox_), "set a voxelgrid size for fpfh")
    ("fpfh.normal_radius", value<float>(&fpfh_normal_r_), "set the normal estimation radius for fpfh")
    ("fpfh.radius", value<float>(&fpfh_r_), "feature estimation radius for fpfh")
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

  ifstream ifs(config_file.c_str());
  if (!ifs)
    cout << "cannot open config file: " << config_file << endl;
  else
  {
    store(parse_config_file(ifs, cfg), vm);
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
void processFPFH(const PointCloud<PointXYZRGBA>::Ptr in,
		 PointCloud<PointXYZRGBA>::Ptr ref_out,
		 PointCloud<PointXYZRGBA>::Ptr fpfh_out)
{
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>());
  PointCloud<FPFHSignature33>::Ptr fpfh(new PointCloud<FPFHSignature33>());
  EvalResults stats((char)RES_FPFH); // create instance to hold FPFH results
  boost::timer t;

  // Passthrough filtering (needs to be done to remove NaNs)
  cout << "FPFH: Pass (with " << in->points.size() << " points)" << endl;
  PassThrough<PointXYZRGBA> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0f, pass_depth_);
  pass.filter(*ref_out);

  // Optional voxelgrid filtering
  if (fpfh_vox_enable_)
  {
    cout << "FPFH: Voxel (with " << ref_out->points.size() << " points)" << endl;
    VoxelGrid<PointXYZRGBA> vox;
    vox.setInputCloud(ref_out);
    vox.setLeafSize(fpfh_vox_, fpfh_vox_, fpfh_vox_);
    vox.filter(*ref_out);
  }

  KdTree<PointXYZRGBA>::Ptr tree(new KdTreeFLANN<PointXYZRGBA>());
  tree->setInputCloud(ref_out);

  // Optional surface smoothing
  if(fpfh_mls_enable_)
  {
    cout << "FPFH: MLS (with " << ref_out->points.size() << " points)" << endl;
    t.restart(); // restart timer
    MovingLeastSquares<PointXYZRGBA, Normal> mls;
    mls.setInputCloud(ref_out);
    mls.setOutputNormals(n);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(fpfh_normal_r_);
    mls.reconstruct(*ref_out);
    cout << "FPFH: flip normals (with " << ref_out->points.size() << " points)" << endl;
    for (size_t i = 0; i < ref_out->points.size(); ++i)
    {
      flipNormalTowardsViewpoint(ref_out->points[i], 0.0f, 0.0f, 0.0f, 
				 n->points[i].normal[0],
				 n->points[i].normal[1],
				 n->points[i].normal[2]);
    }
    stats.t_normal_ = t.elapsed(); // save timer
  }
  else
  {
    cout << "FPFH: Normals (with " << ref_out->points.size() << " points)" << endl;
    t.restart(); // restart timer
    NormalEstimation<PointXYZRGBA, Normal> norm;
    norm.setInputCloud(ref_out);
    norm.setSearchMethod(tree);
    norm.setRadiusSearch(fpfh_normal_r_);
    norm.compute(*n);
    stats.t_normal_ = t.elapsed(); // save timer
  }

  // FPFH estimation
  tree.reset(new KdTreeFLANN<PointXYZRGBA>());
  tree->setInputCloud(ref_out);
  cout << "FPFH: estimation (with " << ref_out->points.size() << " points)" << endl;
  t.restart(); // restart timer
  FPFHEstimation<PointXYZRGBA, Normal, FPFHSignature33> fpfhE;
  fpfhE.setInputCloud(ref_out);
  fpfhE.setInputNormals(n);
  fpfhE.setSearchMethod(tree);
  fpfhE.setRadiusSearch(fpfh_r_);
  fpfhE.compute(*fpfh);
  stats.t_feature_ = t.elapsed(); // save timer
  cout << "FPFH: classification " << endl;
  fpfh_out->width = ref_out->width;
  fpfh_out->height = ref_out->height;
  fpfh_out->points.resize(fpfh_out->width * fpfh_out->height);

  // use KNN classifier if specified
  if (fpfh_knn_values_ != "" && fpfh_knn_labels_ != "")
  {
    t.restart(); // restart timer
    cob_3d_mapping_features::KNNClassifier<FPFHSignature33> knn;
    knn.setKNeighbors(fpfh_knn_k_);
    knn.loadTrainingData(fpfh_knn_values_, fpfh_knn_labels_);

    for (size_t i = 0; i < ref_out->points.size(); i++)
    {
      fpfh_out->points[i].x = ref_out->points[i].x;
      fpfh_out->points[i].y = ref_out->points[i].y;
      fpfh_out->points[i].z = ref_out->points[i].z;

      int res = knn.classify(fpfh->points[i]);

      switch(res)
      {
      case SVM_PLANE:
	fpfh_out->points[i].rgba = LBL_PLANE;
	if ((ref_out->points[i].rgba != LBL_PLANE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_PLANE);
	break;
      case SVM_EDGE_CAV:
	fpfh_out->points[i].rgba = LBL_EDGE;
	if ((ref_out->points[i].rgba != LBL_EDGE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_EDGE);
	break;
      case SVM_EDGE_CVX:
	fpfh_out->points[i].rgba = LBL_EDGE;
	if ((ref_out->points[i].rgba != LBL_EDGE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_EDGE);
	break;      
      case SVM_SPH_CAV:
	fpfh_out->points[i].rgba = LBL_SPH;
	if ((ref_out->points[i].rgba != LBL_SPH) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_SPH);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      case SVM_SPH_CVX:
	fpfh_out->points[i].rgba = LBL_SPH;
	if ((ref_out->points[i].rgba != LBL_SPH) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_SPH);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      case SVM_CYL_CAV:
	fpfh_out->points[i].rgba = LBL_CYL;
	if ((ref_out->points[i].rgba != LBL_CYL) && (ref_out->points[i].rgba != LBL_UNDEF))
	  stats.incFP(I_CYL);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      case SVM_CYL_CVX:
	fpfh_out->points[i].rgba = LBL_CYL;
	if ((ref_out->points[i].rgba != LBL_CYL) && (ref_out->points[i].rgba != LBL_UNDEF))
	  stats.incFP(I_CYL);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      default:
	fpfh_out->points[i].rgba = LBL_UNDEF;
	break;
      }
    
      switch(ref_out->points[i].rgba) 
      {
      case LBL_PLANE:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_PLANE);
	stats.incRef(I_PLANE);
	break;
      case LBL_EDGE:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_EDGE);
	stats.incRef(I_EDGE);
	break;
      case LBL_SPH:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_SPH);
	stats.incRef(I_SPH);
	break;
      case LBL_CYL:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_CYL);
	stats.incRef(I_CYL);
	break;
      default:
	stats.incUndef();
	break;
      }
    }
    stats.t_classifier_ = t.elapsed(); // save timer
  }
  // else use SVM classifier
  else
  {
    t.restart(); // restart timer
    CvSVM svm;
    svm.load(fpfh_svm_model_.c_str());
    cv::Mat fpfh_histo(1, 33, CV_32FC1);

    for (size_t i = 0; i < ref_out->points.size(); i++)
    {
      fpfh_out->points[i].x = ref_out->points[i].x;
      fpfh_out->points[i].y = ref_out->points[i].y;
      fpfh_out->points[i].z = ref_out->points[i].z;

      memcpy(fpfh_histo.ptr<float>(0), fpfh->points[i].histogram, sizeof(fpfh->points[i].histogram));
      float res = svm.predict(fpfh_histo);

      switch((int)res)
      {
      case SVM_PLANE:
	fpfh_out->points[i].rgba = LBL_PLANE;
	if ((ref_out->points[i].rgba != LBL_PLANE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_PLANE);
	break;
      case SVM_EDGE_CAV:
	fpfh_out->points[i].rgba = LBL_EDGE;
	if ((ref_out->points[i].rgba != LBL_EDGE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_EDGE);
	break;
      case SVM_EDGE_CVX:
	fpfh_out->points[i].rgba = LBL_EDGE;
	if ((ref_out->points[i].rgba != LBL_EDGE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_EDGE);
	break;      
      case SVM_SPH_CAV:
	fpfh_out->points[i].rgba = LBL_SPH;
	if ((ref_out->points[i].rgba != LBL_SPH) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_SPH);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      case SVM_SPH_CVX:
	fpfh_out->points[i].rgba = LBL_SPH;
	if ((ref_out->points[i].rgba != LBL_SPH) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	  stats.incFP(I_SPH);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      case SVM_CYL_CAV:
	fpfh_out->points[i].rgba = LBL_CYL;
	if ((ref_out->points[i].rgba != LBL_CYL) && (ref_out->points[i].rgba != LBL_UNDEF))
	  stats.incFP(I_CYL);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      case SVM_CYL_CVX:
	fpfh_out->points[i].rgba = LBL_CYL;
	if ((ref_out->points[i].rgba != LBL_CYL) && (ref_out->points[i].rgba != LBL_UNDEF))
	  stats.incFP(I_CYL);
	if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	  stats.incTPnEnP();
	break;
      default:
	fpfh_out->points[i].rgba = LBL_UNDEF;
	break;
      }
    
      switch(ref_out->points[i].rgba) 
      {
      case LBL_PLANE:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_PLANE);
	stats.incRef(I_PLANE);
	break;
      case LBL_EDGE:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_EDGE);
	stats.incRef(I_EDGE);
	break;
      case LBL_SPH:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_SPH);
	stats.incRef(I_SPH);
	break;
      case LBL_CYL:
	if (fpfh_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_CYL);
	stats.incRef(I_CYL);
	break;
      default:
	stats.incUndef();
	break;
      }
    }
    stats.t_classifier_ = t.elapsed(); // save timer
  }
  stats.points_ = fpfh_out->size();

  // print results to console
  cout << "\n\t+-------------------------------------" << endl;
  cout << "[FPFH]--| " << stats.getLabel() << endl;
  cout << "\t+-------------------------------------" << endl;

  stats.printAllToConsole("");
  if (folder_out_ != "")
  {
    io::savePCDFileASCII(folder_out_ + "fpfh.pcd", *fpfh);
    io::savePCDFileASCII(folder_out_ + "fpfh_ref.pcd", *ref_out);
  }

  if (log_prefix_ != "")
    stats.writeTimerLog(log_prefix_ + "fpfh_timer.log");
}

/*! @brief runs the whole processing pipeline for PC features
 *
 * @note At the moment the evaluation results will be printed to console.
 *
 * @param[in] in the labeled input point cloud
 * @param[out] ref_out the reference point cloud after the preprocessing steps
 * @param[out] pc_out the labeled point cloud after the classifing process
 */
void processPC(const PointCloud<PointXYZRGBA>::Ptr in,
	       PointCloud<PointXYZRGBA>::Ptr ref_out,
	       PointCloud<PointXYZRGBA>::Ptr pc_out)
{
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>());
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>());
  EvalResults stats((char)RES_PC); // create an instance for PC results
  boost::timer t;
  
  // passthrough filtering (needed to remove NaNs)
  cout << "PC: Pass (with " << in->points.size() << " points)" << endl;
  PassThrough<PointXYZRGBA> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0f, pass_depth_);
  pass.filter(*ref_out);

  // Optional voxelgrid filtering
  if (pc_vox_enable_)
  {
    cout << "PC: Voxel (with " << ref_out->points.size() << " points)" << endl;
    VoxelGrid<PointXYZRGBA> vox;
    vox.setInputCloud(ref_out);
    vox.setLeafSize(pc_vox_, pc_vox_, pc_vox_);
    vox.filter(*ref_out);
  }

  KdTree<PointXYZRGBA>::Ptr tree(new KdTreeFLANN<PointXYZRGBA>());
  tree->setInputCloud(ref_out);

  // Optional surface smoothing
  if(pc_mls_enable_)
  {
    cout << "PC: MLS (with " << ref_out->points.size() << " points)" << endl;
    t.restart(); // restart timer
    MovingLeastSquares<PointXYZRGBA, Normal> mls;
    mls.setInputCloud(ref_out);
    mls.setOutputNormals(n);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(pc_normal_r_);
    mls.reconstruct(*ref_out);
    cout << "PC: flip normals (with " << ref_out->points.size() << " points)" << endl;
    for (size_t i = 0; i < ref_out->points.size(); ++i)
    {
      flipNormalTowardsViewpoint(ref_out->points[i], 0.0f, 0.0f, 0.0f, 
				 n->points[i].normal[0],
				 n->points[i].normal[1],
				 n->points[i].normal[2]);
    }
    stats.t_normal_ = t.elapsed(); // save timer
  }
  else
  {
    cout << "PC: Normals (with " << ref_out->points.size() << " points)" << endl;
    t.restart(); //restart timer
    NormalEstimation<PointXYZRGBA, Normal> norm;
    norm.setInputCloud(ref_out);
    norm.setSearchMethod(tree);
    norm.setRadiusSearch(pc_normal_r_);
    norm.compute(*n);
    stats.t_normal_ = t.elapsed(); // save timer
  }

  // estimate PC
  tree.reset(new KdTreeFLANN<PointXYZRGBA>());
  tree->setInputCloud(ref_out);
  cout << "PC: estimation (with " << ref_out->points.size() << " points)" << endl;
  t.restart(); // restart timer
  PrincipalCurvaturesEstimation<PointXYZRGBA, Normal, PrincipalCurvatures> pcE;
  pcE.setInputCloud(ref_out);
  pcE.setInputNormals(n);
  pcE.setSearchMethod(tree);
  pcE.setRadiusSearch(pc_r_);
  pcE.compute(*pc);
  stats.t_feature_ = t.elapsed(); // save timer

  cout << "PC: classification " << endl;
  pc_out->width = ref_out->width;
  pc_out->height = ref_out->height;
  pc_out->points.resize(pc_out->width * pc_out->height);

  // apply rules to PC results
  t.restart(); // restart timer
  for (size_t i = 0; i < ref_out->points.size(); i++)
  {
    pc_out->points[i].x = ref_out->points[i].x;
    pc_out->points[i].y = ref_out->points[i].y;
    pc_out->points[i].z = ref_out->points[i].z;

    if (pc->points[i].pc1 < pc_c_max_th_lower_)
    {
      pc_out->points[i].rgba = LBL_PLANE;
      if ((ref_out->points[i].rgba != LBL_PLANE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	stats.incFP(I_PLANE);
    }
    else if (pc->points[i].pc1 < pc_max_min_ratio_ * pc->points[i].pc2 &&
	     pc->points[i].pc1 < pc_c_max_th_sphere_upper_)
    {
      pc_out->points[i].rgba = LBL_SPH;
      if ((ref_out->points[i].rgba != LBL_SPH) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	stats.incFP(I_SPH);
      if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	stats.incTPnEnP();
    }
    else if (pc->points[i].pc1 > pc_c_max_th_cylinder_upper_)
    {
      pc_out->points[i].rgba = LBL_EDGE;
      if ((ref_out->points[i].rgba != LBL_EDGE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	stats.incFP(I_EDGE);
    }
    else
    {
      pc_out->points[i].rgba = LBL_CYL;
      if ((ref_out->points[i].rgba != LBL_CYL) && (ref_out->points[i].rgba != LBL_UNDEF))
	stats.incFP(I_CYL);
      if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	stats.incTPnEnP();
    }

    switch(ref_out->points[i].rgba) 
    {
    case LBL_PLANE:
      if (pc_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_PLANE);
      stats.incRef(I_PLANE);
      break;
    case LBL_EDGE:
      if (pc_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_EDGE);
      stats.incRef(I_EDGE);
      break;
    case LBL_SPH:
      if (pc_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_SPH);
      stats.incRef(I_SPH);
      break;
    case LBL_CYL:
      if (pc_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_CYL);
      stats.incRef(I_CYL);
      break;
    default:
      stats.incUndef();
      break;
    }
  }
  stats.t_classifier_ = t.elapsed(); // save timer
  stats.points_ = pc_out->size();

  // print results to console
  cout << "\n\t+-------------------------------------" << endl;
  cout<< "[PC]----| " << stats.getLabel() << endl;
  cout << "\t+-------------------------------------" << endl;
  cout<< "\t| c_max_upper_cyl/sph:\t" 
      << pc_c_max_th_cylinder_upper_<<" / "<<pc_c_max_th_sphere_upper_ <<"\n"
      << "\t| c_max_lower:\t\t" << pc_c_max_th_lower_ <<"\n"
      << "\t| c_max_min_ratio:\t\t" << pc_max_min_ratio_ << endl;
  cout << "\t+-------------------------------------" << endl;

  stats.printAllToConsole("");
  if (folder_out_ != "")
  {
    io::savePCDFileASCII(folder_out_ + "pc.pcd", *pc);
    io::savePCDFileASCII(folder_out_ + "pc_out.pcd", *pc_out);
    io::savePCDFileASCII(folder_out_ + "pc_ref.pcd", *ref_out);
  }
  if (log_prefix_ != "")
    stats.writeTimerLog(log_prefix_ + "pc_timer.log");
}

/*! @brief runs the whole processing pipeline for RSD features
 *
 * @note At the moment the evaluation results will be printed to console.
 *
 * @param[in] in the labeled input point cloud
 * @param[out] ref_out the reference point cloud after the preprocessing steps
 * @param[out] rsd_out the labeled point cloud after the classifing process
 */
void processRSD(const PointCloud<PointXYZRGBA>::Ptr in, 
		PointCloud<PointXYZRGBA>::Ptr ref_out,
		PointCloud<PointXYZRGBA>::Ptr rsd_out)
{
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>());
  PointCloud<PrincipalRadiiRSD>::Ptr rsd(new PointCloud<PrincipalRadiiRSD>());
  EvalResults stats((char)RES_RSD); // create an instance for RSD results
  boost::timer t;

  // passthrough filtering (needed to remove NaNs)
  cout << "RSD: Pass (with " << in->points.size() << " points)" << endl;
  PassThrough<PointXYZRGBA> pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0f, pass_depth_);
  pass.filter(*ref_out);

  // optional voxelgrid filtering
  if (rsd_vox_enable_)
  {
    cout << "RSD: Voxel (with " << ref_out->points.size() << " points)" << endl;
    VoxelGrid<PointXYZRGBA> vox;
    vox.setInputCloud(ref_out);
    vox.setLeafSize(rsd_vox_, rsd_vox_, rsd_vox_);
    vox.filter(*ref_out);
  }

  KdTree<PointXYZRGBA>::Ptr tree(new KdTreeFLANN<PointXYZRGBA>());
  tree->setInputCloud(ref_out);

  // optional surface smoothing
  if(rsd_mls_enable_)
  {
    cout << "RSD: MLS (with " << ref_out->points.size() << " points)" << endl;
    t.restart(); // restart timer
    MovingLeastSquares<PointXYZRGBA, Normal> mls;
    mls.setInputCloud(ref_out);
    mls.setOutputNormals(n);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(rsd_normal_r_);
    mls.reconstruct(*ref_out);
    cout << "RSD: flip normals (with " << ref_out->points.size() << " points)" << endl;
    for (size_t i = 0; i < ref_out->points.size(); ++i)
    {
      flipNormalTowardsViewpoint(ref_out->points[i], 0.0f, 0.0f, 0.0f, 
				 n->points[i].normal[0],
				 n->points[i].normal[1],
				 n->points[i].normal[2]);
    }
    stats.t_normal_ = t.elapsed(); // save timer
  }
  else
  {
    cout << "RSD: Normals (with " << ref_out->points.size() << " points)" << endl;
    t.restart(); // restart timer
    NormalEstimation<PointXYZRGBA, Normal> norm;
    norm.setInputCloud(ref_out);
    norm.setSearchMethod(tree);
    norm.setRadiusSearch(rsd_normal_r_);
    norm.compute(*n);
    stats.t_normal_ = t.elapsed(); // save timer
  }

  tree->setInputCloud(ref_out);

  // RSD estimation
  cout << "RSD: estimation (with " << ref_out->points.size() << " points)" << endl;
  t.restart(); // restart timer
  RSDEstimation<PointXYZRGBA, Normal, PrincipalRadiiRSD> rsdE;
  rsdE.setInputCloud(ref_out);
  rsdE.setInputNormals(n);
  rsdE.setSearchMethod(tree);
  rsdE.setPlaneRadius(rsd_r_max_);
  rsdE.setRadiusSearch(rsd_r_);
  rsdE.compute(*rsd);
  stats.t_feature_ = t.elapsed(); // save timer

  cout << "RSD: classification " << endl;
  rsd_out->width = ref_out->width;
  rsd_out->height = ref_out->height;
  rsd_out->points.resize(rsd_out->width * rsd_out->height);

  // apply RSD rules for classification
  t.restart(); // restart timer
  for (size_t i = 0; i < ref_out->points.size(); i++)
  {
    rsd_out->points[i].x = ref_out->points[i].x;
    rsd_out->points[i].y = ref_out->points[i].y;
    rsd_out->points[i].z = ref_out->points[i].z;

    if (rsd->points[i].r_min > rsd_r_min_th_upper_)
    {
      rsd_out->points[i].rgba = LBL_PLANE;
      if ((ref_out->points[i].rgba != LBL_PLANE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	stats.incFP(I_PLANE);
    }
    else if (rsd->points[i].r_max < rsd_max_min_ratio_ * rsd->points[i].r_min &&
      rsd->points[i].r_min > rsd_r_min_th_sphere_lower_)
    {
      rsd_out->points[i].rgba = LBL_SPH;
      if ((ref_out->points[i].rgba != LBL_SPH) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	stats.incFP(I_SPH);
      if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	stats.incTPnEnP();
    }
    else if (rsd->points[i].r_min < rsd_r_min_th_cylinder_lower_)
    {
      rsd_out->points[i].rgba = LBL_EDGE;
      if ((ref_out->points[i].rgba != LBL_EDGE) && (ref_out->points[i].rgba != LBL_UNDEF)) 
	stats.incFP(I_EDGE);
    }
    else
    {
      rsd_out->points[i].rgba = LBL_CYL;
      if ((ref_out->points[i].rgba != LBL_CYL) && (ref_out->points[i].rgba != LBL_UNDEF))
	stats.incFP(I_CYL);
      if (ref_out->points[i].rgba == LBL_SPH || ref_out->points[i].rgba == LBL_CYL)
	stats.incTPnEnP();
    }

    switch(ref_out->points[i].rgba) 
    {
    case LBL_PLANE:
      if (rsd_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_PLANE);
      stats.incRef(I_PLANE);
      break;
    case LBL_EDGE:
      if (rsd_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_EDGE);
      stats.incRef(I_EDGE);
      break;
    case LBL_SPH:
      if (rsd_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_SPH);
      stats.incRef(I_SPH);
      break;
    case LBL_CYL:
      if (rsd_out->points[i].rgba != ref_out->points[i].rgba) stats.incFN(I_CYL);
      stats.incRef(I_CYL);
      break;
    default:
      stats.incUndef();
      break;
    }
  }
  stats.t_classifier_ = t.elapsed(); // save timer
  stats.points_ = rsd_out->size();

  // print results to console
  cout<< "\n\t+-------------------------------------" << endl;
  cout<< "[RSD]---| " << stats.getLabel() << endl;
  cout << "\t+-------------------------------------" << endl;
  cout<< "\t| r_min_lower_cyl/sph:\t" 
      << rsd_r_min_th_cylinder_lower_<<" / "<< rsd_r_min_th_sphere_lower_ <<"\n"
      << "\t| r_min_upper:\t\t" << rsd_r_min_th_upper_ <<"\n"
      << "\t| r_max_min_ratio:\t\t" << rsd_max_min_ratio_ << endl;
  cout << "\t+-------------------------------------" << endl;

  stats.printAllToConsole("");
  if (folder_out_ != "")
  {
    io::savePCDFileASCII(folder_out_ + "rsd_out.pcd", *rsd_out);
    io::savePCDFileASCII(folder_out_ + "rsd.pcd", *rsd);
    io::savePCDFileASCII(folder_out_ + "rsd_ref.pcd", *ref_out);
  }
  if (log_prefix_ != "")
    stats.writeTimerLog(log_prefix_ + "rsd_timer.log");
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGBA>::Ptr p_raw(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGBA>::Ptr p_rsd_out(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGBA>::Ptr p_rsd_ref(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGBA>::Ptr p_pc_out(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGBA>::Ptr p_pc_ref(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGBA>::Ptr p_fpfh_out(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGBA>::Ptr p_fpfh_ref(new PointCloud<PointXYZRGBA>());

  PCDReader r;
  if(r.read(file_in_, *p_raw) == -1)
    cout << "Could not read file " << file_in_ << endl;
  cout << "Read pcd file \"" << file_in_ << "\" (Points: " << p_raw->points.size() << ", width: " 
       << p_raw->width << ", height: " << p_raw->height << ")" << endl;

  // visualization::PCLVisualizer x;
  // ColorHdlRGBA col_hdl(p_raw);
  // x.addPointCloud<PointXYZRGBA>(p_raw, col_hdl);

  // while(!x.wasStopped())
  // {
  //   x.spinOnce(100);
  //   usleep(100000);
  // }

  if (rsd_enable_)
    processRSD(p_raw, p_rsd_ref, p_rsd_out);
  if (pc_enable_)
    processPC(p_raw, p_pc_ref, p_pc_out);
  if (fpfh_enable_)
    processFPFH(p_raw, p_fpfh_ref, p_fpfh_out);

  if (vis_enable_)
  {
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
    ColorHdlRGBA col_hdl1(p_rsd_ref);
    v->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
    v->setBackgroundColor(255,255,255, v1);
    v->addPointCloud<PointXYZRGBA>(p_rsd_ref, col_hdl1, "raw", v1);

    // 2:
    int v2(0);
    ColorHdlRGBA col_hdl2(p_fpfh_out);
    v->createViewPort(0.0, 0.0, 0.5, 0.5, v2);
    v->setBackgroundColor(255,255,255, v2);
    v->addPointCloud<PointXYZRGBA>(p_fpfh_out, col_hdl2, "fpfh", v2);
  
    // 3:
    int v3(0);
    ColorHdlRGBA col_hdl3(p_rsd_out);
    v->createViewPort(0.5, 0.5, 1.0, 1.0, v3);
    v->setBackgroundColor(255,255,255, v3);
    v->addPointCloud<PointXYZRGBA>(p_rsd_out, col_hdl3, "rsd", v3);
  
    // 4:
    int v4(0);
    ColorHdlRGBA col_hdl4(p_pc_out);
    v->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
    v->setBackgroundColor(255,255,255, v4);
    v->addPointCloud<PointXYZRGBA>(p_pc_out, col_hdl4, "pc", v4);

    while(!v->wasStopped())
    {
      v->spinOnce(100);
      usleep(100000);
    }
    return(0);
  }
}
