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

#include <cob_3d_evaluation_features/label_results.h>

using namespace pcl;
using namespace std;

vector<string> scenes_;
string folder_, config_file_;
bool en_pc, en_rsd;

float c_low_min, c_low_max, c_low_steps, c_high_min, c_high_max, c_high_steps;
float c_r1_min, c_r1_max, c1_steps, c_r2_min, c_r2_max, c2_steps;

float r_low_min, r_low_max, r_low_steps, r_high_min, r_high_max, r_high_steps;
float r_r1_min, r_r1_max, r1_steps, r_r2_min, r_r2_max, r2_steps;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("config,c", value<string>(&config_file_), "cfg file")
    ("folder", value<string>(&folder_), "set folder")
    ("scenes", value<vector<string> >(&scenes_), "name of used scenes")
    ("pc,P", "enable PC")
    ("rsd,R", "enable RSD")

    ("c_low_min", value<float>(&c_low_min), "")
    ("c_low_max", value<float>(&c_low_max), "")
    ("c_low_steps", value<float>(&c_low_steps), "")
    ("c_high_min", value<float>(&c_high_min), "")
    ("c_high_max", value<float>(&c_high_max), "")
    ("c_high_steps", value<float>(&c_high_steps), "")
    ("c_r1_min", value<float>(&c_r1_min), "")
    ("c_r1_max", value<float>(&c_r1_max), "")
    ("c_r2_min", value<float>(&c_r2_min), "")
    ("c_r2_max", value<float>(&c_r2_max), "")
    ("c1_steps", value<float>(&c1_steps), "")
    ("c2_steps", value<float>(&c2_steps), "")

    ("r_low_min", value<float>(&r_low_min), "")
    ("r_low_max", value<float>(&r_low_max), "")
    ("r_low_steps", value<float>(&r_low_steps), "")
    ("r_high_min", value<float>(&r_high_min), "")
    ("r_high_max", value<float>(&r_high_max), "")
    ("r_high_steps", value<float>(&r_high_steps), "")
    ("r_r1_min", value<float>(&r_r1_min), "")
    ("r_r1_max", value<float>(&r_r1_max), "")
    ("r_r2_min", value<float>(&r_r2_min), "")
    ("r_r2_max", value<float>(&r_r2_max), "")
    ("r1_steps", value<float>(&r1_steps), "")
    ("r2_steps", value<float>(&r2_steps), "")
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
    ifstream ifs2( (folder_ + "evaluate_settings.cfg").c_str() );
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
  if (vm.count("pc")) en_pc = true;
  if (vm.count("rsd")) en_rsd = true;
}

int main (int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<PrincipalRadiiRSD>::Ptr rsd(new PointCloud<PrincipalRadiiRSD>);

  for (size_t i = 0; i < scenes_.size(); i++)
  {
    vector<string> pc_pcds, rsd_pcds;
    getAllPcdFilesInDirectory(folder_ + "0_pc/" + scenes_[i] + "/", pc_pcds);
    getAllPcdFilesInDirectory(folder_ + "0_rsd/" + scenes_[i] + "/", rsd_pcds);

    cout << "Scene " << scenes_[i] << ": Found " << pc_pcds.size() << " pcd files for PC" << endl;
    cout << "Scene " << scenes_[i] << ": Found " << rsd_pcds.size() << " pcd files for RSD" << endl;

    if (en_pc)
    {
      string logfile = folder_ + "results/" + scenes_[i] + "/res_pc_" +
	cob_3d_mapping_common::createTimestamp() + ".csv";

      cob_3d_mapping_common::writeHeader(logfile,"c_low","c_high","c_r1(cyl/sph)","c_r2(cor/edge)");

      for (size_t j = 0; j < pc_pcds.size(); j++)
      {
	bool is_mls;
	string rn, rf;
	cob_3d_mapping_common::parseFileName(pc_pcds[j], rn, rf, is_mls);
	io::loadPCDFile<PointXYZRGB>(folder_+"normals/"+scenes_[i]+"/mls_"+scenes_[i]+
				     "_"+rn+"rn.pcd",*p);
	rn = "0."+rn;
	rf = "0."+rf;
	cout << scenes_[i] <<";"<< rn <<";"<< rf <<";"<< is_mls << endl;
	io::loadPCDFile<PrincipalCurvatures>(folder_+"0_pc/"+scenes_[i]+"/"+pc_pcds[j], *pc);


	int exp_rgb, pre_rgb;
	float c_max,c_min;

	for (float c_low = c_low_min; c_low <= c_low_max; c_low += c_low_steps)
	{
	  for (float c_high = c_high_min; c_high <= c_high_max; c_high += c_high_steps)
	  {
	    for (float c_r1 = c_r1_min; c_r1 <= c_r1_max; c_r1 += c1_steps)
	    {
	      for (float c_r2 = c_r2_min; c_r2 <= c_r2_max; c_r2 += c2_steps)
	      {
		cout << c_low <<" | "<< c_high <<" | "<< c_r1 <<" | "<< c_r2 <<endl;
		// new instance class for holding evaluation results:
		cob_3d_mapping_common::LabelResults stats(rn,rf,is_mls);
		for (size_t idx = 0; idx < p->points.size(); idx++)
		{
		  exp_rgb = *reinterpret_cast<int*>(&p->points[idx].rgb); // expected label
		  c_max = pc->points[idx].pc1;
		  c_min = pc->points[idx].pc2;

		  if ( c_max < c_low )
		  {
		    pre_rgb = LBL_PLANE;
		    if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_PLANE]++;
		  }
		  else if (c_max > c_high)
		  {
		    if (c_max < c_r2 * c_min)
		    {
		      pre_rgb = LBL_COR;
		      if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[EVAL_COR]++;
		    }
		    else
		    {
		      pre_rgb = LBL_EDGE;
		      if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGE]++;
		    }
		    // special case:  combined class for corner and edge
		    if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF)
		      stats.fp[EVAL_EDGECORNER]++;
		  }
		  else
		  {
		    if (c_max < c_r1 * c_min)
		    {
		      pre_rgb = LBL_SPH;
		      if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[EVAL_SPH]++;
		    }
		    else
		    {
		      pre_rgb = LBL_CYL;
		      if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CYL]++;
		    }
		    // special case:  combined class for sphere and cylinder
		    if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF)
		      stats.fp[EVAL_CURVED]++;
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
		}

		stats.calcResults();
		stats.writeToFile(logfile, c_low, c_high, c_r1, c_r2);
	      }
	    }
	  }
	}
      }
    }

    if (en_rsd)
    {
      string logfile = folder_ + "results/" + scenes_[i] + "/res_rsd_" +
	cob_3d_mapping_common::createTimestamp() + ".csv";
      cob_3d_mapping_common::writeHeader(logfile,"r_low","r_high","r_r1(cyl/sph)","r_r2(cor/edge)");

      for (size_t j = 0; j < pc_pcds.size(); j++)
      {
	bool is_mls;
	string rn, rf;
	cob_3d_mapping_common::parseFileName(rsd_pcds[j], rn, rf, is_mls);
	io::loadPCDFile<PointXYZRGB>(folder_+"normals/"+scenes_[i]+"/mls_"+scenes_[i]+
				     "_"+rn+"rn.pcd",*p);
	rn = "0."+rn;
	rf = "0."+rf;
	cout << scenes_[i] <<";"<< rn <<";"<< rf <<";"<< is_mls << endl;
	io::loadPCDFile<PrincipalRadiiRSD>(folder_+"0_rsd/"+scenes_[i]+"/"+rsd_pcds[j], *rsd);

	int exp_rgb, pre_rgb;
	float r_max,r_min;

	for (float r_low = r_low_min; r_low <= r_low_max; r_low += r_low_steps)
	{
	  for (float r_high = r_high_min; r_high <= r_high_max; r_high += r_high_steps)
	  {
	    for (float r_r1 = r_r1_min; r_r1 <= r_r1_max; r_r1 += r1_steps)
	    {
	      for (float r_r2 = r_r2_min; r_r2 <= r_r2_max; r_r2 += r2_steps)
	      {
		cout << r_low <<" | "<< r_high <<" | "<< r_r1 <<" | "<< r_r2 <<endl;
		// new instance class for holding evaluation results:
		cob_3d_mapping_common::LabelResults stats(rn,rf,is_mls);
		for (size_t idx = 0; idx < p->points.size(); idx++)
		{
		  exp_rgb = *reinterpret_cast<int*>(&p->points[idx].rgb); // expected label
		  r_max = rsd->points[idx].r_max;
		  r_min = rsd->points[idx].r_min;

		  if ( r_min > r_high )
		  {
		    pre_rgb = LBL_PLANE;
		    if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_PLANE]++;
		  }
		  else if (r_min < r_low)
		  {
		    if (r_max < r_r2 * r_min)
		    {
		      pre_rgb = LBL_COR;
		      if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[EVAL_COR]++;
		    }
		    else
		    {
		      pre_rgb = LBL_EDGE;
		      if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[EVAL_EDGE]++;
		    }
		    // special case:  combined class for corner and edge
		    if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF)
		      stats.fp[EVAL_EDGECORNER]++;
		  }
		  else
		  {
		    if (r_max < r_r1 * r_min)
		    {
		      pre_rgb = LBL_SPH;
		      if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[EVAL_SPH]++;
		    }
		    else
		    {
		      pre_rgb = LBL_CYL;
		      if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[EVAL_CYL]++;
		    }
		    // special case:  combined class for sphere and cylinder
		    if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF)
		      stats.fp[EVAL_CURVED]++;
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
		}

		stats.calcResults();
		stats.writeToFile(logfile, r_low, r_high, r_r1, r_r2);
	      }
	    }
	  }
	}
      }
    }
  }

  return 0;
}
