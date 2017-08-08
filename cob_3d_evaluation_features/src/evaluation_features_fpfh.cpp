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
 * \date Date of creation: 01/2012
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

#include <fstream>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <cob_3d_evaluation_features/label_results.h>
#include <opencv2/ml/ml.hpp>

using namespace pcl;
using namespace std;
typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string rgba_, fpfh_, svm_;
string logfile_;
bool write_header_ = false, vis_ = false;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("logfile", value<string>(&logfile_), "set logfile for results")
    ("fpfh", value<string>(&fpfh_), "fpfh features pcd")
    ("rgba", value<string>(&rgba_), "rgba pcd")
    ("svm", value<string>(&svm_), "svm")
    ("header,H", "create new logfile with header")
    ("visualize,v", "show results in visualizer")
    ;

  positional_options_description p_opt;
  p_opt.add("rgba", 1).add("fpfh", 1).add("svm", 1).add("logfile", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || argc <= 1)
  {
    cout << options << endl;
    exit(0);
  }
  if (vm.count("header")) write_header_ = true;
  if (vm.count("visualize")) vis_ = true;
}

int main (int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p_out(new PointCloud<PointXYZRGB>);
  PointCloud<FPFHSignature33>::Ptr fpfh(new PointCloud<FPFHSignature33>);

  if (write_header_)
  {
    if (logfile_ != "") cob_3d_mapping_common::writeHeader(logfile_,"svm");
    else cout << cob_3d_mapping_common::writeHeader("","svm") << endl;;
  }

  bool is_mls;
  string rn, rf;
  cob_3d_mapping_common::parseFileName(fpfh_, rn, rf, is_mls);
  rn = "0."+rn;
  rf = "0."+rf;
  //cout << "load data" << endl;
  io::loadPCDFile<PointXYZRGB>(rgba_,*p);
  io::loadPCDFile<FPFHSignature33>(fpfh_, *fpfh);
  *p_out = *p;
  //cout << "load svm" << endl;
#if CV_MAJOR_VERSION == 2
  CvSVM svm;
  svm.load(svm_.c_str());
#else
  cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load(svm_.c_str());
#endif
  cv::Mat fpfh_histo(1, 33, CV_32FC1);

  int exp_rgb, pre_rgb, predict;

  // new instance class for holding evaluation results:
  cob_3d_mapping_common::LabelResults stats(rn,rf,is_mls);
  //cout << "start labeling" << endl;
  for (size_t idx = 0; idx < p->points.size(); idx++)
  {
    //cout << idx << ": ";
    exp_rgb = *reinterpret_cast<int*>(&p->points[idx].rgb); // expected label
    //cout << exp_rgb << " / ";
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
    p_out->points[idx].rgb = *reinterpret_cast<float*>(&pre_rgb);
  }
  //cout << "calc stats" << endl;
  stats.calcResults();
  //cout << "write stats" << endl;
  if (logfile_ != "")
    stats.writeToFile(logfile_, svm_);
  else
    cout << stats.writeToFile("", svm_) << endl;;

  if (vis_)
  {
    ColorHdlRGB col_hdl(p_out);
    visualization::PCLVisualizer v;
    v.setBackgroundColor(255,255,255);
    v.addPointCloud<PointXYZRGB>(p_out, col_hdl ,"fpfh");
    while(!v.wasStopped())
    {
      v.spinOnce(100);
      usleep(100000);
    }
  }

  return 0;
}
