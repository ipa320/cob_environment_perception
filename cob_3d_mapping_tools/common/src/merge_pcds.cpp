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

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

string out, in_f;
vector<string> in_pcds;


void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("out,o", value<string> (&out), "output pcd file name")
    ("folder,f", value<string> (&in_f), "input folder containing all pcd files to merge")
    ("pcds,p", value<vector<string> > (&in_pcds), "list of input pcds to merge")
    ;

  positional_options_description p_opt;
  p_opt.add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("out"))
  {
    cout << "\nTool to merge pcd files (of all types) specified by a folder "
	 << "or given as a list to a single file\n\n";
    cout << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  pcl::PCLPointCloud2 cloud_out;
  pcl::PCLPointCloud2 cloud_in;
  pcl::PCLPointCloud2 cloud_tmp;
  if (in_f != "") 
  {
    vector<string> tmp_in;
    getAllPcdFilesInDirectory(in_f, tmp_in);
    for(size_t i=0; i<tmp_in.size(); i++)
    {
      in_pcds.push_back(in_f + tmp_in[i]);
    }
  }
  cout << "PCD files to merge: " << in_pcds.size() << endl;
  io::loadPCDFile(in_pcds[0], cloud_out);
  for (size_t i=1; i<in_pcds.size(); i++)
  {
    io::loadPCDFile(in_pcds[i], cloud_in);
    cout << "loaded " << cloud_in.width * cloud_in.height << " points..." << endl;
    concatenatePointCloud (cloud_in,cloud_out,cloud_tmp);
    cout << "copied " << cloud_tmp.width * cloud_tmp.height << " points..." << endl;
    cloud_out = cloud_tmp;

  }
  io::savePCDFile(out, cloud_out);
  cout << "saved " << cloud_out.width * cloud_out.height << " points!" << endl;

  return 0;
}
