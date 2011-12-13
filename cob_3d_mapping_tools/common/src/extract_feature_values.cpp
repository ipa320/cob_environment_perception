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

string in_label;
string in_feature;
string out;

bool plane;
bool edge;
bool cyl;
bool sph;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in_label,l", value<string>(&in_label)->default_value(""),
     "pcd file containting rgba labeled points")
    ("in_feature,f", value<string>(&in_feature)->default_value(""),
     "pcd file containting feature values")
    ("out,o", value<string>(&out)->default_value(""),
     "output folder")
    ("plane,p", value<bool>(&plane)->default_value(true), 
     "extract points labeled as plane")
    ("edge,e", value<bool>(&edge)->default_value(true), 
     "extract points labeled as edge")
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
    cout << "extract feature values (RSD,PC,FPFH) of a classified point cloud "
	 << "and store them separated for each class" << endl;
    cout << options << endl;
    exit(0);
  }
}

template<typename PointT> 
void extractIndices(const string & file_in, 
		    const string & folder, 
		    const string & prefix,
		    const vector< vector<int> > & indices)
{
  PointCloud<PointT> p;
  PointCloud<PointT> p_ex;

  io::loadPCDFile<PointT>(file_in, p);
  if (indices[0].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[0], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_plane.pcd", p_ex);
  }
  if (indices[1].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[1], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_edge.pcd", p_ex);
  }
  if (indices[2].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[2], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_cyl.pcd", p_ex);
  }
  if (indices[3].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[3], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_sph.pcd", p_ex);
  }
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGBA>::Ptr p_label (new PointCloud<PointXYZRGBA>);
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);

  vector<int> idx_plane;
  vector<int> idx_edge;
  vector<int> idx_cyl;
  vector<int> idx_sph;

  vector< vector<int> > indices(4, vector<int>());
  int mode = 0;

  io::loadPCDFile(in_label, *p_label);
  io::loadPCDFile(in_feature, *cloud);
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

  for (size_t i = 0; i < p_label->size(); ++i)
  {
    switch(p_label->points[i].rgba)
    {
    case LBL_PLANE:
      indices[0].push_back(i);
      break;

    case LBL_EDGE:
      indices[1].push_back(i);
      break;

    case LBL_CYL:
      indices[2].push_back(i);
      break;

    case LBL_SPH:
      indices[3].push_back(i);
      break;

    default:
      break;
    }
  }

  if(!plane)
    indices[0].clear();
  if(!edge)
    indices[1].clear();
  if(!cyl)
    indices[2].clear();
  if(!sph)
    indices[3].clear();

  switch(mode)
  {
  case 1:
    extractIndices<PrincipalRadiiRSD>(in_feature, out, "rsd", indices);
    break;

  case 2:
    extractIndices<PrincipalCurvatures>(in_feature, out, "pc", indices);
    break;

  case 3:
    extractIndices<FPFHSignature33>(in_feature, out, "fpfh", indices);
    break;

  default:
    break;
  }

  
  return 1;
}
