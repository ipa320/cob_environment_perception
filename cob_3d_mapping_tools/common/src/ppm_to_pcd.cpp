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

// Boost:
#include <boost/program_options.hpp>
// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "cob_3d_mapping_tools/io.h"

using namespace std;
using namespace pcl;

vector<string> file_i(2, "");
string file_o = "";
bool remove_undef = false;


void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("rm_undef,R", "remove points labeled as undefined from point cloud")
    ("in", value< vector<string> >(&file_i), "input files, first ppm, second pcd")
    ("out", value<string> (&file_o), "output pcd file")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 2).add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << "Reads a .ppm image and maps the color values on a point cloud" << endl;
    cout << options << endl;
    exit(0);
  }
  if (vm.count("rm_undef"))
  {
    remove_undef = true;
  }
  if (file_o == "")
  {
    cout << "no output file defined " << endl << options << endl;
    exit(0);
  }
  if (file_i[0] == "" || file_i[1] == "")
  {
    cout << "no input files defined " << endl << options << endl;
    exit(0);
  }
}

/*!
 * @brief Reads a .ppm image and maps the color values on a point cloud
 */
int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZ>::Ptr p(new PointCloud<PointXYZ>());
  PointCloud<PointXYZRGB>::Ptr pc(new PointCloud<PointXYZRGB>());

  PCDReader r;
  if (r.read(file_i[1], *p) == -1) return(0);
  copyPointCloud<PointXYZ, PointXYZRGB>(*p, *pc);
  pc->height = p->height;
  pc->width = p->width;
  cob_3d_mapping_tools::PPMReader ppmR;
  if (ppmR.mapRGB(file_i[0], *pc, remove_undef) == -1)
  {
    cout << "Mapping error" << endl;
    return(0);
  }
  cout << "Mapped colors to \"" << file_o << "\" (Points: " << pc->points.size() << ", width: "
       << pc->width << ", height: " << pc->height << ")" << endl;
  PCDWriter w;
  io::savePCDFileASCII(file_o, *pc);
  return(0);
}
