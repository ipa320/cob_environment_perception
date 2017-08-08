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

#include <boost/program_options.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>

#include <opencv2/ml/ml.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_common/label_defines.h>

using namespace std;
using namespace pcl;

string file_plane;
string file_edge;
string file_corner;
string file_sphere;
string file_cylinder;
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
    cout << "\t  \"fpfh_plane.pcd\"" << endl;
    cout << "\t  \"fpfh_edge.pcd\"" << endl;
    cout << "\t  \"fpfh_cor.pcd\"" << endl;
    cout << "\t  \"fpfh_sph.pcd\"" << endl;
    cout << "\t  \"fpfh_cyl.pcd\"" << endl;
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
      if(string::npos != files[i].rfind("fpfh_plane"))
	file_plane = files[i];
      else if (string::npos != files[i].rfind("fpfh_edge"))
	file_edge = files[i];
      else if (string::npos != files[i].rfind("fpfh_cor"))
	file_corner = files[i];
      else if (string::npos != files[i].rfind("fpfh_sph"))
	file_sphere = files[i];
      else if (string::npos != files[i].rfind("fpfh_cyl"))
	file_cylinder = files[i];
    }
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<FPFHSignature33>::Ptr p_plane (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr p_edge (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr p_corner (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr p_cylinder (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr p_sphere (new PointCloud<FPFHSignature33>);

  PCDReader reader;
  cout << "Read "<< file_plane << " for plane." << endl;
  reader.read(file_plane, *p_plane);
  cout << "Read "<< file_edge << " for edge." << endl;
  reader.read(file_edge, *p_edge);
  cout << "Read "<< file_corner << " for corner." << endl;
  reader.read(file_corner, *p_corner);
  cout << "Read "<< file_sphere << " for sph." << endl;
  reader.read(file_sphere, *p_sphere);
  cout << "Read "<< file_cylinder << " for cyl." << endl;
  reader.read(file_cylinder, *p_cylinder);

  int plane_samples, edge_samples, corner_samples, sphere_samples, cylinder_samples, sample_count;

  if (svm_samples == 0)
  {
    plane_samples = p_plane->height * p_plane->width;
    edge_samples = p_edge->height * p_edge->width;
    sphere_samples =  p_sphere->height * p_sphere->width;
    cylinder_samples = p_cylinder->height * p_cylinder->width;
    corner_samples = p_corner->height * p_corner->width;
  }
  else
  {
    plane_samples = edge_samples = corner_samples = sphere_samples = cylinder_samples = svm_samples;
  }
  sample_count = plane_samples + edge_samples + corner_samples + sphere_samples + cylinder_samples;
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
      row[j] = p_plane->points[i].histogram[j];
    }
  }

  // Read FPFHs for Edge Categorie
  i_end += plane_samples;
  for (int i = 0; i < edge_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_EDGE;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = p_edge->points[i].histogram[j];
    }
  }

  i_end += edge_samples;
  for (int i = 0; i < corner_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_COR;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = p_corner->points[i].histogram[j];
    }
  }

  // Read FPFHs for Sphere Categorie
  i_end += corner_samples;
  for (int i = 0; i < sphere_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_SPH;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = p_sphere->points[i].histogram[j];
    }
  }

  // Read FPFHs for Cylinder Categorie
  i_end += sphere_samples;
  for (int i = 0; i < cylinder_samples; i ++)
  {
    train_data_classes.at<float>(i + i_end) = SVM_CYL;
    row = train_data.ptr<float>(i + i_end);
    for(int j = 0; j < 33; j++)
    {
      row[j] = p_cylinder->points[i].histogram[j];
    }
  }

  std::cout << "Begin training..." << std::endl;
#if CV_MAJOR_VERSION == 2
  cv::SVM svm;
  cv::SVMParams params;
  params.svm_type = cv::SVM::C_SVC;//NU_SVC;//C_SVC;
  params.kernel_type = cv::SVM::RBF;
  params.nu = 0.5;
  params.C = svm_c;
  params.gamma = svm_gamma;
  // Define iteration termination criteria:
  params.term_crit.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
  params.term_crit.max_iter = 1000;
  params.term_crit.epsilon = 0.00001;
  std::cout << "Starting training process with " << sample_count << " samples." << std::endl;
  if(svm_k == 0) 
    svm.train(train_data, train_data_classes, cv::Mat(), cv::Mat(), params);
  else 
    svm.train_auto(train_data, train_data_classes, cv::Mat(), cv::Mat(), params, svm_k);
  string file_xml = file_folder + "SVM_fpfh_rng_g_c.xml";
  svm.save(file_xml.c_str());
#else
  cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
  svm->setType(cv::ml::SVM::C_SVC);	//NU_SVC;//C_SVC;
  svm->setKernel(cv::ml::SVM::RBF);
  svm->setNu(0.5);
  svm->setC(svm_c);
  svm->setGamma(svm_gamma);
  // Define iteration termination criteria:
  cv::TermCriteria term_crit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, 0.00001);
  svm->setTermCriteria(term_crit);
  std::cout << "Starting training process with " << sample_count << " samples." << std::endl;
  cv::Ptr<cv::ml::TrainData> train_data_and_labels = cv::ml::TrainData::create(train_data, cv::ml::ROW_SAMPLE, train_data_classes);
  if(svm_k == 0)
    svm->train(train_data_and_labels);
  else
    svm->trainAuto(train_data_and_labels, svm_k);
  string file_xml = file_folder + "SVM_fpfh_rng_g_c.xml";
  svm->save(file_xml.c_str());

#endif
  //cout << "C: " << params.C << " - Gamma: " << svm_gamma << " / " << params.gamma << endl;
  cout << "Training process completed." << endl;
}
