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
 * \date Date of creation: 11/2011
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

#include <stdlib.h>
#include <fstream>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/rsd.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh_omp.h>

using namespace pcl;
using namespace std;

string folder_, config_file_;
vector<string> scenes_;
float r_min_, r_max_, r_step_, limit_;

string fl2label(float f, size_t precision)
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
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("config,c", value<string>(&config_file_), "cfg file")
    ("folder", value<string>(&folder_), "set folder")
    ("scenes", value<vector<string> >(&scenes_), "name of used scenes")
    ("r_min", value<float>(&r_min_)->default_value(0.02f), "radius to start with")
    ("r_max", value<float>(&r_max_)->default_value(0.03f), "radius to end with")
    ("r_step", value<float>(&r_step_)->default_value(0.005f), "radius step width")
    ("z_limit", value<float>(&limit_)->default_value(3.5f), "z distance limit")
    ;

  positional_options_description p_opt;
  p_opt.add("folder", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("folder"))
  {
    cout << options << endl;
    exit(0);
  }
  ifstream ifs(config_file_.c_str());
  if (!ifs)
  {
    ifstream ifs2( (folder_ + "generate_settings.cfg").c_str() );
    if (!ifs2)
    {
      cout << "cannot open config file!" << endl;
    }
    else
    {
      store(parse_config_file(ifs2, options), vm);
      notify(vm);
    }
  }
  else
  {
    store(parse_config_file(ifs, options), vm);
    notify(vm);
  }
  /*
  if (vm.count("scenes"))
  {
    cout << "Selected scenes are: "
	 << vm["scenes"].as< vector<string> >() << "\n";
  }
  */
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGB>::Ptr in(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr n_mls(new PointCloud<Normal>);
  PointCloud<PointXYZRGB>::Ptr p_mls(new PointCloud<PointXYZRGB>);
  PointCloud<FPFHSignature33>::Ptr fpfh(new PointCloud<FPFHSignature33>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<PrincipalRadiiRSD>::Ptr rsd(new PointCloud<PrincipalRadiiRSD>);
  
  for (size_t i = 0; i < scenes_.size(); i++)
  {
    io::loadPCDFile<PointXYZRGB>(folder_ + "raw_labeled/" + scenes_[i] + ".pcd", *in);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0f, limit_);
    pass.filter(*in);

    #ifdef PCL_VERSION_COMPARE //fuerte
      pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
    #else //electric
      pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
    #endif
    tree->setInputCloud(in);

    for (float r_n = r_min_; r_n <= r_max_; r_n += r_step_)
    {
      string str_rn = fl2label(r_n,3) + "rn";

      NormalEstimationOMP<PointXYZRGB, Normal> norm;
      norm.setNumberOfThreads(1);
      norm.setInputCloud(in);
      norm.setSearchMethod(tree);
      norm.setRadiusSearch(r_n);
      norm.compute(*n);

      #ifdef PCL_MINOR_VERSION
      #if PCL_MINOR_VERSION  >=7
      std::cerr << "current implementation of mls doesn't work with pcl1.7" << std::endl;
      exit(-1);
      #else
      MovingLeastSquares<PointXYZRGB, Normal> mls;
      mls.setInputCloud(in);
      mls.setOutputNormals(n_mls);
      mls.setPolynomialFit(true);
      mls.setPolynomialOrder(2);
      mls.setSearchMethod(tree);
      mls.setSearchRadius(r_n);
      mls.reconstruct(*p_mls);
      #endif
      #endif

      for (size_t p = 0; p < n_mls->points.size(); ++p)
      {
        flipNormalTowardsViewpoint(p_mls->points[p], 0.0f, 0.0f, 0.0f, 
                                   n_mls->points[p].normal[0],
                                   n_mls->points[p].normal[1],
                                   n_mls->points[p].normal[2]);
      }

      io::savePCDFileASCII<PointXYZRGB>(folder_+"normals/"+scenes_[i]+
                                        "/mls_"+scenes_[i]+"_"+str_rn+".pcd",*p_mls);

#ifdef PCL_VERSION_COMPARE //fuerte
      pcl::search::KdTree<PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<PointXYZRGB>());
#else //electric
      pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree_mls (new pcl::KdTreeFLANN<PointXYZRGB> ());
#endif
      tree_mls->setInputCloud(p_mls);
      
      for (float r_f = r_n; r_f <= r_max_; r_f += r_step_)
      {
        cout << "scene: " << scenes_[i] << "\tnormals: " << r_n << "\tfeatures: " << r_f << endl;
        string str_rf = fl2label(r_f,3) + "rf";

        FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33> fpfh_est;
        fpfh_est.setNumberOfThreads(1);
        fpfh_est.setInputCloud(in);
        fpfh_est.setInputNormals(n);
        fpfh_est.setSearchMethod(tree);
        fpfh_est.setRadiusSearch(r_f);
        fpfh_est.compute(*fpfh);
        // filename example: 
        //    <folder_>/fpfh/kitchen01/fpfh_kitchen01_020rn_025rf.pcd
        //    <folder_>/fpfh/kitchen02/fpfh_kitchen02_030rnmls_060rf.pcd
        io::savePCDFileASCII<FPFHSignature33>(folder_ + "0_fpfh/" + scenes_[i] + 
                                              "/fpfh_" + scenes_[i] +"_"+str_rn+"_"+str_rf+".pcd",
                                              *fpfh);

        FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33> fpfh_est_mls;
        fpfh_est_mls.setNumberOfThreads(1);
        fpfh_est_mls.setInputCloud(p_mls);
        fpfh_est_mls.setInputNormals(n_mls);
        fpfh_est_mls.setSearchMethod(tree_mls);
        fpfh_est_mls.setRadiusSearch(r_f);
        fpfh_est_mls.compute(*fpfh);
        io::savePCDFileASCII<FPFHSignature33>(folder_ + "0_fpfh/" + scenes_[i] + 
                                              "/fpfh_" + scenes_[i] +"_"+str_rn+"mls_"+str_rf+".pcd",
                                              *fpfh);

        PrincipalCurvaturesEstimation<PointXYZRGB, Normal, PrincipalCurvatures> pc_est;
        pc_est.setInputCloud(in);
        pc_est.setInputNormals(n);
        pc_est.setSearchMethod(tree);
        pc_est.setRadiusSearch(r_f);
        pc_est.compute(*pc);
        io::savePCDFileASCII<PrincipalCurvatures>(folder_ + "0_pc/" + scenes_[i] + 
                                                  "/pc_" + scenes_[i] +"_"+str_rn+"_"+str_rf+".pcd",
                                                  *pc);

        PrincipalCurvaturesEstimation<PointXYZRGB, Normal, PrincipalCurvatures> pc_est_mls;
        pc_est_mls.setInputCloud(p_mls);
        pc_est_mls.setInputNormals(n_mls);
        pc_est_mls.setSearchMethod(tree_mls);
        pc_est_mls.setRadiusSearch(r_f);
        pc_est_mls.compute(*pc);
        io::savePCDFileASCII<PrincipalCurvatures>(folder_ + "0_pc/" + scenes_[i] + 
                                                  "/pc_" + scenes_[i] +"_"+str_rn+"mls_"+str_rf+".pcd",
                                                  *pc);

        RSDEstimation<PointXYZRGB, Normal, PrincipalRadiiRSD> rsd_est;
        rsd_est.setInputCloud(in);
        rsd_est.setInputNormals(n);
        rsd_est.setSearchMethod(tree);
        rsd_est.setPlaneRadius(0.5);
        rsd_est.setRadiusSearch(r_f);
        rsd_est.compute(*rsd);
        io::savePCDFileASCII<PrincipalRadiiRSD>(folder_ + "0_rsd/" + scenes_[i] + 
                                                "/rsd_" + scenes_[i] +"_"+str_rn+"_"+str_rf+".pcd",
                                                *rsd);

        RSDEstimation<PointXYZRGB, Normal, PrincipalRadiiRSD> rsd_est_mls;
        rsd_est_mls.setInputCloud(p_mls);
        rsd_est_mls.setInputNormals(n_mls);
        rsd_est_mls.setSearchMethod(tree_mls);
        rsd_est_mls.setPlaneRadius(0.5);
        rsd_est_mls.setRadiusSearch(r_f);
        rsd_est_mls.compute(*rsd);
        io::savePCDFileASCII<PrincipalRadiiRSD>(folder_ + "0_rsd/" + scenes_[i] + 
                                                "/rsd_" + scenes_[i] +"_"+str_rn+"mls_"+str_rf+".pcd",
                                                *rsd);
      }
    }
  }

  return 0;
}
