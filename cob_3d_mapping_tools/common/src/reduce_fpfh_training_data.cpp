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


#include "cob_3d_mapping_features/most_discriminating_data_points.h"
#include <boost/program_options.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

string in_, out_;
int k_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in", value<string>(&in_), "input fpfh pcd")
    ("out", value<string>(&out_), "output fpfh pcd")
    ("intervals,k", value<int>(&k_)->default_value(100), "k means value")
    ;

  positional_options_description p_opt;
  p_opt.add("in",1).add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<FPFHSignature33>::Ptr f_in (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr f_out (new PointCloud<FPFHSignature33>);

  io::loadPCDFile<FPFHSignature33>(in_, *f_in);
  cout << "loaded fpfh" << endl;
  vector<vector<float> > d_in;
  vector<vector<float> > d_out;

  d_in.resize(f_in->size());
  for (size_t n=0;n<f_in->size();n++)
  {
    d_in.at(n) = vector<float>(f_in->points[n].histogram,
			       f_in->points[n].histogram +
			       sizeof(f_in->points[n].histogram) / sizeof(float));
  }
  cout << "copied fpfh" << endl;
  cob_3d_mapping_features::MostDiscriminatingDataPoints md;
  md.setInputData(&d_in);
  md.setK(k_);
  md.computeDataPoints(&d_out);
  cout << "computed kmeans" << endl;

  f_out->points.resize(k_);

  for (size_t k=0; k<k_; k++)
  {
    for (size_t m=0;m<33;m++) f_out->points[k].histogram[m] = d_out.at(k).at(m);
  }
  cout << "saved fpfh" << endl;
  io::savePCDFileASCII<FPFHSignature33>(out_, *f_out);
  return (0);
}
