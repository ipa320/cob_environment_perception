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
 * FPFH_SVM_trainer.cpp
 *
 *  Created on: 27.05.2011
 *      Author: goa-sf
 */

#include <boost/program_options.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>

#include <opencv2/ml/ml.hpp>

#include <cob_3d_mapping_common/label_defines.h>

using namespace std;
using namespace pcl;

string file_plane;
string file_edge_vex;
string file_edge_cav;
string file_sphere_vex;
string file_sphere_cav;
string file_cylinder_vex;
string file_cylinder_cav;
string file_folder = "";

double svm_gamma;
double svm_c;
int svm_k;

int svm_samples;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  
  options_description cmd_line("Options");
  cmd_line.add_options()
    ("help", "produce help messsage")
    ("svm_gamma,g", value<double>(&svm_gamma)->default_value(1.0), 
     "Training parameter gamma")
    ("svm_c,c", value<double>(&svm_c)->default_value(1.0),
     "Training parameter c")
    ("svm_k,k", value<int>(&svm_k)->default_value(0),
     "Set k_fold != 0 to use train_auto")
    ("folder,f", value<string>(&file_folder),
     "path to folder containing a pcd file for each class. " 
     "If this is defined all seperatly defined files will be ignored.")
    ("svm_max_samples,s", value<int>(&svm_samples)->default_value(0), 
     "maximal number of samples per class. (0 for all)")
    ;
  variables_map vm;
  store(command_line_parser(argc, argv).options(cmd_line).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << "Reads files containing FPFH features and creates a SVM model" << endl;
    cout << "The following files from the defined folder containing the strings are used:" << endl;
    cout << "\t  \"plane\"" << endl;
    cout << "\t  \"edge_convex_\"" << endl;
    cout << "\t  \"edge_concave_\"" << endl;
    cout << "\t  \"sphere_convex_\"" << endl;
    cout << "\t  \"sphere_concave_\"" << endl;
    cout << "\t  \"cylinder_convex_\"" << endl;
    cout << "\t  \"cylinder_concave_\"" << endl;
    cout << cmd_line << endl;
    exit(0);
  }

  if (vm.count("folder"))
  {
    vector<string> files;
    //    PCDReader r;
    sensor_msgs::PointCloud2 c;
    getAllPcdFilesInDirectory(file_folder, files);
    for (unsigned int i = 0; i < files.size(); i++)
    {
      files[i] = file_folder + files[i];
      // if(!r.read(files[i], c))
      // 	continue;
      // if(getFieldIndex(c, "histogram") == -1) 
      // 	continue;
      if(string::npos != files[i].rfind("plane_"))
	file_plane = files[i];
      else if (string::npos != files[i].rfind("edge_convex_"))
	file_edge_vex = files[i];
      else if (string::npos != files[i].rfind("edge_concave_"))
	file_edge_cav = files[i];
      else if (string::npos != files[i].rfind("sphere_convex_"))
	file_sphere_vex = files[i];
      else if (string::npos != files[i].rfind("sphere_concave_"))
	file_sphere_cav = files[i];
      else if (string::npos != files[i].rfind("cylinder_convex_"))
	file_cylinder_vex = files[i];
      else if (string::npos != files[i].rfind("cylinder_concave_"))
	file_cylinder_cav = files[i];
    }
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<FPFHSignature33> pc_plane, pc_edge_vex, pc_edge_cav, pc_sphere_vex, pc_sphere_cav, pc_cylinder_vex, pc_cylinder_cav;
  PCDReader reader;
  cout << "Read "<< file_plane << " for plane." << endl;
  reader.read(file_plane, pc_plane);
  cout << "Read "<< file_edge_vex << " for edge vex." << endl;
  reader.read(file_edge_vex, pc_edge_vex);
  cout << "Read "<< file_edge_cav << " for edge cave." << endl;
  reader.read(file_edge_cav, pc_edge_cav);
  cout << "Read "<< file_sphere_vex << " for sph vex." << endl;
  reader.read(file_sphere_vex, pc_sphere_vex);
  cout << "Read "<< file_sphere_cav << " for sph cave." << endl;
  reader.read(file_sphere_cav, pc_sphere_cav);
  cout << "Read "<< file_cylinder_vex << " for cyl vex." << endl;
  reader.read(file_cylinder_vex, pc_cylinder_vex);
  cout << "Read "<< file_cylinder_cav << " for cyl cave." << endl;
  reader.read(file_cylinder_cav, pc_cylinder_cav);

  int plane_samples, edge_vex_samples, edge_cav_samples, sphere_vex_samples, cylinder_vex_samples, sphere_cav_samples, cylinder_cav_samples, sample_count;

  if (svm_samples == 0)
  {
    plane_samples = pc_plane.height * pc_plane.width;
    edge_vex_samples = pc_edge_vex.height * pc_edge_vex.width;
    sphere_vex_samples =  pc_sphere_vex.height * pc_sphere_vex.width;
    cylinder_vex_samples = pc_cylinder_vex.height * pc_cylinder_vex.width;
    edge_cav_samples = pc_edge_cav.height * pc_edge_cav.width;
    sphere_cav_samples =  pc_sphere_cav.height * pc_sphere_cav.width;
    cylinder_cav_samples = pc_cylinder_cav.height * pc_cylinder_cav.width;
  }
  else
  {
    plane_samples = edge_vex_samples = edge_cav_samples = sphere_cav_samples = cylinder_cav_samples = sphere_vex_samples = cylinder_vex_samples = svm_samples;
  }
  sample_count = plane_samples + edge_vex_samples + edge_cav_samples + sphere_vex_samples + cylinder_vex_samples + sphere_cav_samples + cylinder_cav_samples;
  cout << "Found " << sample_count << " samples." << endl;
  cv::Mat train_data(sample_count, 33, CV_32FC1);
  cv::Mat train_data_classes(sample_count, 1, CV_32FC1);
  
  // Read FPFHs for Plane Categorie
  float* row;
  int i_end = 0;
  for (int i = 0; i < plane_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_PLANE;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_plane.points[i].histogram[j];
    }
  }

  // Read FPFHs for Edge Categorie
  i_end += plane_samples;
  for (int i = 0; i < edge_vex_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_EDGE_CVX;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_edge_vex.points[i].histogram[j];
    }
  }

  i_end += edge_vex_samples;
  for (int i = 0; i < edge_cav_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_EDGE_CAV;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_edge_cav.points[i].histogram[j];
    }
  }

  // Read FPFHs for Sphere Categorie
  i_end += edge_cav_samples;
  for (int i = 0; i < sphere_vex_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_SPH_CVX;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_sphere_vex.points[i].histogram[j];
    }
  }
  i_end += sphere_vex_samples;
  for (int i = 0; i < sphere_cav_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_SPH_CAV;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_sphere_cav.points[i].histogram[j];
    }
  }

  // Read FPFHs for Cylinder Categorie
  i_end += sphere_cav_samples;
  for (int i = 0; i < cylinder_vex_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_CYL_CVX;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_cylinder_vex.points[i].histogram[j];
    }
  }
  i_end += cylinder_vex_samples;
  for (int i = 0; i < cylinder_cav_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_CYL_CAV;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = pc_cylinder_cav.points[i].histogram[j];
    }
  }

  cout << "Begin training..." << endl;
  CvSVM svm;
  CvSVMParams params;
  params.svm_type = CvSVM::C_SVC;//NU_SVC;//C_SVC;
  params.kernel_type = CvSVM::RBF;
  params.nu = 0.5;
  params.C = svm_c;
  params.gamma = svm_gamma;
  // Define iteration termination criteria:
  params.term_crit.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
  params.term_crit.max_iter = 200;
  params.term_crit.epsilon = 0.00001;
  cout << "Starting training process with " << sample_count << " samples." << endl;
  if(svm_k == 0) 
    svm.train(train_data, train_data_classes, cv::Mat(), cv::Mat(), params);
  else 
    svm.train_auto(train_data, train_data_classes, cv::Mat(), cv::Mat(), params, svm_k);
  string file_xml = file_folder + "SVM_fpfh_rng_g_c.xml";
  svm.save(file_xml.c_str());
  //cout << "C: " << params.C << " - Gamma: " << svm_gamma << " / " << params.gamma << endl;
  cout << "Training process completed." << endl;
}
