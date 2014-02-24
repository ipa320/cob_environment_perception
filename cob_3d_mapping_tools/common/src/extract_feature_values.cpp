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
 *  ROS package name: cob_3d_mapping_tools
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

/*!
 * @brief extract feature values (RSD,PC,FPFH) of a classified point cloud 
 *   and store them separated for each class
 */

#include <cob_3d_mapping_common/label_defines.h>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

vector<string> in_label;
vector<string> in_feature;
string out;

bool plane, edge, cor, cyl, sph;


void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in_label,l", value<vector<string> >(&in_label),
     "pcd file containting rgb labeled points")
    ("in_feature,f", value<vector<string> >(&in_feature),
     "pcd file containting feature values")
    ("out,o", value<string>(&out)->default_value(""),
     "output folder")
    ("plane,p", value<bool>(&plane)->default_value(true), 
     "extract points labeled as plane")
    ("edge,e", value<bool>(&edge)->default_value(true), 
     "extract points labeled as edge")
    ("corner,o", value<bool>(&cor)->default_value(true), 
     "extract points labeled as corner")
    ("cyl,c", value<bool>(&cyl)->default_value(true), 
     "extract points labeled as cylinder")
    ("sph,s", value<bool>(&sph)->default_value(true), 
     "extract points labeled as sphere")
    ;

  positional_options_description p_opt;
  p_opt.add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << "extract feature values (RSD, PC or FPFH) of a classified point cloud "
	 << "and store them separated for each class" << endl;
    cout << options << endl;
    exit(0);
  }
  if (in_label.size() != in_feature.size() || in_label.size() == 0)
    cout << "please define an equal number of label and feature files" << endl;
}

template<typename PointT> 
void extractIndices(const PointCloud<PointT>& points, 
		    const string& folder, 
		    const string& prefix,
		    const vector< vector<int> >& indices)
{
  PointCloud<PointT> p_ex;

  if (indices[0].size() != 0)
  {
    copyPointCloud<PointT>(points, indices[0], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_plane.pcd", p_ex);
  }
  if (indices[1].size() != 0)
  {
    copyPointCloud<PointT>(points, indices[1], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_edge.pcd", p_ex);
  }
  if (indices[2].size() != 0)
  {
    copyPointCloud<PointT>(points, indices[2], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_cor.pcd", p_ex);
  }
  if (indices[3].size() != 0)
  {
    copyPointCloud<PointT>(points, indices[3], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_cyl.pcd", p_ex);
  }
  if (indices[4].size() != 0)
  {
    copyPointCloud<PointT>(points, indices[4], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_sph.pcd", p_ex);
  }
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p_label (new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p (new PointCloud<PointXYZRGB>);

  PointCloud<PrincipalRadiiRSD>::Ptr f1 (new PointCloud<PrincipalRadiiRSD>);
  PointCloud<PrincipalCurvatures>::Ptr f2 (new PointCloud<PrincipalCurvatures>);
  PointCloud<FPFHSignature33>::Ptr f3 (new PointCloud<FPFHSignature33>);

  PointCloud<PrincipalRadiiRSD>::Ptr f1b (new PointCloud<PrincipalRadiiRSD>);
  PointCloud<PrincipalCurvatures>::Ptr f2b (new PointCloud<PrincipalCurvatures>);
  PointCloud<FPFHSignature33>::Ptr f3b (new PointCloud<FPFHSignature33>);
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

  vector<int> idx_plane;
  vector<int> idx_edge;
  vector<int> idx_cor;
  vector<int> idx_cyl;
  vector<int> idx_sph;

  vector< vector<int> > indices(5, vector<int>());
  int mode = 0;

  io::loadPCDFile(in_feature[0], *cloud);
  if (getFieldIndex(*cloud, "r_max") != -1)
  {
    cout << "Point cloud with RSD features selected." << endl;
    mode = 1;
  }
  else if (getFieldIndex(*cloud, "pc1") != -1)
  {
    cout << "Point cloud with principal curvatures selected." << endl;
    mode = 2;
  }
  else if (getFieldIndex(*cloud, "fpfh") != -1)
  {
    cout << "Point cloud with FPFH selected." << endl;
    mode = 3;
  }

  for (size_t i=0;i<in_label.size();i++)
  {
    io::loadPCDFile(in_label[i], *p);
    *p_label += *p;
    switch (mode)
    {
    case 1:
      io::loadPCDFile(in_feature[i], *f1b);
      *f1 += *f1b;
      break;
    case 2:
      io::loadPCDFile(in_feature[i], *f2b);
      *f2 += *f2b;
      break;
    case 3:
      io::loadPCDFile(in_feature[i], *f3b);
      *f3 += *f3b;
      break;
    default:
      exit(0);
    }
  }

  cout<<"selected "<<p_label->size()<<" points from "<<in_label.size()<<" pcd files."<<endl;
  cout<<"selected "<<p_label->points.size()<<" points from "<<in_label.size()<<" pcd files."<<endl;
  cout<<"selected "<<f3->size()<<" feature points"<<endl;

  uint32_t rgb;
  for (size_t i = 0; i < p_label->size(); ++i)
  {
    //cout << i << ": " << p_label->points[i].rgb;
    rgb = *reinterpret_cast<int*>(&(p_label->points[i].rgb));
    //cout << " | " << rgb << endl;
    switch(rgb)
    {
    case LBL_PLANE:
      indices[0].push_back(i);
      break;

    case LBL_EDGE:
      indices[1].push_back(i);
      break;

    case LBL_COR:
      indices[2].push_back(i);
      break;

    case LBL_CYL:
      indices[3].push_back(i);
      break;

    case LBL_SPH:
      indices[4].push_back(i);
      break;

    default:
      break;
    }
  }

  if(!plane)
    indices[0].clear();
  if(!edge)
    indices[1].clear();
  if(!cor)
    indices[2].clear();
  if(!cyl)
    indices[3].clear();
  if(!sph)
    indices[4].clear();

  switch(mode)
  {
  case 1:
    extractIndices<PrincipalRadiiRSD>(*f1, out, "rsd", indices);
    break;
  case 2:
    extractIndices<PrincipalCurvatures>(*f2, out, "pc", indices);
    break;
  case 3:
    extractIndices<FPFHSignature33>(*f3, out, "fpfh", indices);
    break;
  default:
    break;
  }
  
  return 1;
}
