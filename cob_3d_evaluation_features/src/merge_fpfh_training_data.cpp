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
 * @brief script to merge fpfh PCD files from multiple objects into a single file
 *     for each class.
 */

#include <boost/program_options.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>

//#include <cob_3d_features/svm_class_defines.h>

using namespace std;
using namespace pcl;

string outfolder_;
vector<string> folder_in_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  
  options_description cmd_line("Options");
  cmd_line.add_options()
    ("help", "produce help messsage")
    ("out,o", value<string>(&outfolder_), "output folder")
    ("in,i", value<vector<string> >(&folder_in_), "list folders containing input files")
    ;

  positional_options_description p_opt;
  p_opt.add("in", -1);

  variables_map vm;
  store(command_line_parser(argc, argv)
	.options(cmd_line)
	.positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("in"))
  {
    cout << "Script to merge fpfh PCD files from multiple objects into a single file for each class."
	 << endl;
    cout << "Searches for files containing the strings:" << endl;
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
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<FPFHSignature33> plane, e_vex, e_cav, c_vex, c_cav, s_vex, s_cav, tmp;
  PCDReader r;
  for (size_t j = 0; j < folder_in_.size(); j++)
  {
    vector<string> files;
    getAllPcdFilesInDirectory(folder_in_[j], files);
    for (size_t i = 0; i < files.size(); i++)
    {
      files[i] = folder_in_[j] + files[i];
      r.read(files[i], tmp);
      if(string::npos != files[i].rfind("plane"))
	plane += tmp;
      else if (string::npos != files[i].rfind("edge_convex_"))
	e_vex += tmp;
      else if (string::npos != files[i].rfind("edge_concave_"))
	e_cav += tmp;
      else if (string::npos != files[i].rfind("sphere_convex_"))
	s_vex += tmp;
      else if (string::npos != files[i].rfind("sphere_concave_"))
	s_cav += tmp;	
      else if (string::npos != files[i].rfind("cylinder_convex_"))
	c_vex += tmp;
      else if (string::npos != files[i].rfind("cylinder_concave_"))
	c_cav += tmp;
      tmp.clear();
    }
  }

  PCDWriter w;
  w.write(outfolder_ + "fpfh_plane_.pcd", plane);
  w.write(outfolder_ + "fpfh_edge_convex_.pcd", e_vex);
  w.write(outfolder_ + "fpfh_cylinder_convex_.pcd", c_vex);
  w.write(outfolder_ + "fpfh_sphere_convex_.pcd", s_vex);
  w.write(outfolder_ + "fpfh_edge_concave_.pcd", e_cav);
  w.write(outfolder_ + "fpfh_cylinder_concave_.pcd", c_cav);
  w.write(outfolder_ + "fpfh_sphere_concave_.pcd", s_cav);

  return 1;
}
