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
 * Date of creation: 01/2012
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

#include <fstream>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/label_defines.h"

#define NUM_LBL 7

using namespace pcl;
using namespace std;

class SceneResults
{
public:
  SceneResults(const string rn, const string rf, bool mls) : rn_(rn), rf_(rf), mls_(mls), undef(0)
  {
    for (size_t i = 0; i < NUM_LBL; ++i)
    {
      exp[i] = 0;
      fp[i] = 0;
      fn[i] = 0;
      tp[i] = 0;
      tn[i] = 0;
      prec[i] = 0.0;
      rec[i] = 0.0;
      f1[i] = 0.0;
    }
  }

  void calcResults()
  {
    all = 0;
    for(size_t i = 0; i < NUM_LBL; ++i)
    {
      all = exp[I_PLANE]+exp[I_EDGE]+exp[I_SPH]+exp[I_CYL]+exp[I_COR];
    }
    for(size_t i = 0; i < NUM_LBL; ++i)
    {
      tp[i] = exp[i] - fn[i];
      tn[i] = all - exp[i] - fp[i];
      prec[i] = (float)tp[i] / (tp[i] + fp[i]);
      rec[i] = (float)tp[i] / (tp[i] + fn[i]);
      f1[i] = 2.0f * prec[i] * rec[i] / (prec[i] + rec[i]);
    }
  }

  void writeToFile(const string file,
		   const float p1 = 0.0, 
		   const float p2 = 0.0,
		   const float p3 = 0.0,
		   const float p4 = 0.0)
  {
    fstream f;
    f.open(file.c_str(), fstream::out | fstream::app);
    f << rn_ <<";"<< rf_ <<";"<< mls_ <<";"
      << p1 <<";"<< p2 <<";"<< p3 <<";"<< p4 <<";"
      << all <<";"<< undef <<";";

    for(size_t i = 0; i < NUM_LBL; ++i)
    {
      f << exp[i] <<";"<< tp[i] <<";"<< fp[i] <<";"<< fn[i] <<";"
	<< prec[i] <<";"<< rec[i] <<";"<< f1[i] <<";";
    }

    f << endl;
    f.close();
  }

public:
  int exp [NUM_LBL];
  int fp [NUM_LBL];
  int fn [NUM_LBL];
  int tp [NUM_LBL];
  int tn [NUM_LBL];
  int undef;
  int all;  
  float prec [NUM_LBL];
  float rec [NUM_LBL];
  float f1 [NUM_LBL];

private:
  string rn_;
  string rf_;
  bool mls_;
};

vector<string> scenes_;
string folder_, config_file_;
bool en_pc, en_rsd, en_fpfh;

float c_low_min, c_low_max, c_low_steps, c_high_min, c_high_max, c_high_steps;
float c_r1_min, c_r1_max, c1_steps, c_r2_min, c_r2_max, c2_steps;

float r_low_min, r_low_max, r_low_steps, r_high_min, r_high_max, r_high_steps;
float r_r1_min, r_r1_max, r1_steps, r_r2_min, r_r2_max, r2_steps;

void parseFileName(string file, string &rn, string &rf, bool &is_mls)
{
  string tmp = "";
  if (string::npos != file.rfind("rnmls_"))
  {
    is_mls = true;
    rn = tmp + file[file.length() - 18] + file[file.length() - 17] + file[file.length() - 16];
  }
  else
  {
    is_mls = false;
    rn = tmp + file[file.length() - 15] + file[file.length() - 14] + file[file.length() - 13];
  }
  rf = tmp + file[file.length() - 9] + file[file.length() - 8] + file[file.length() - 7];

  return;
}

string createTimestamp()
{
  struct tm *t;
  time_t rawtime;
  time(&rawtime);
  t = localtime(&rawtime);
  stringstream stamp;
  stamp << (t->tm_year+1900);
  if (t->tm_mon+1 < 10) stamp << "0";
  stamp << t->tm_mon+1;
  if (t->tm_mday < 10) stamp << "0";
  stamp << t->tm_mday << "_";
  if (t->tm_hour < 10) stamp << "0";
  stamp << t->tm_hour;
  if (t->tm_min < 10) stamp << "0";
  stamp << t->tm_min;
  if (t->tm_sec < 10) stamp << "0";
  stamp << t->tm_sec;

  return stamp.str();
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
    ("pc,P", "enable PC")
    ("rsd,R", "enable RSD")
    ("fpfh,F", "enable FPFH")

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
  if (vm.count("fpfh")) en_fpfh = true;
}

int main (int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<FPFHSignature33>::Ptr fpfh(new PointCloud<FPFHSignature33>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<PrincipalRadiiRSD>::Ptr rsd(new PointCloud<PrincipalRadiiRSD>);

  for (size_t i = 0; i < scenes_.size(); i++)
  {
    vector<string> pc_pcds, rsd_pcds, fpfh_pcds;
    getAllPcdFilesInDirectory(folder_ + "0_pc/" + scenes_[i] + "/", pc_pcds);
    getAllPcdFilesInDirectory(folder_ + "0_rsd/" + scenes_[i] + "/", rsd_pcds);
    getAllPcdFilesInDirectory(folder_ + "0_fpfh/" + scenes_[i] + "/", fpfh_pcds);
    
    cout << "Scene " << scenes_[i] << ": Found " << pc_pcds.size() << " pcd files for PC" << endl;
    cout << "Scene " << scenes_[i] << ": Found " << rsd_pcds.size() << " pcd files for RSD" << endl;
    cout << "Scene " << scenes_[i] << ": Found " << fpfh_pcds.size() << " pcd files for FPFH" << endl;

    if (en_pc)
    {
      string logfile = folder_ + "results/" + scenes_[i] + "/res_pc_" + createTimestamp() + ".csv";
      fstream f;
      f.open(logfile.c_str(), fstream::out);
      f << "rn;"<<"rf;"<<"mls;"<<"c_low;"<<"c_high;"<<"c_r1(cyl/sph);"<<"c_r2(cor/edge);"
	<< "all;"<<"undef;"
	<< "exp(Plane);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Edge);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Sphere);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Cylinder);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Corner);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(EdgeCorner);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Curved);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"<< endl;
      f.close();

      for (size_t j = 0; j < pc_pcds.size(); j++)
      {
	bool is_mls;
	string rn, rf;
	parseFileName(pc_pcds[j], rn, rf, is_mls);
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
		SceneResults stats(rn,rf,is_mls);
		for (size_t idx = 0; idx < p->points.size(); idx++)
		{
		  exp_rgb = *reinterpret_cast<int*>(&p->points[idx].rgb); // expected label
		  c_max = pc->points[idx].pc1;
		  c_min = pc->points[idx].pc2;
		  
		  if ( c_max < c_low )
		  {
		    pre_rgb = LBL_PLANE;
		    if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[I_PLANE]++;
		  }
		  else if (c_max > c_high)
		  {
		    if (c_max < c_r2 * c_min)
		    {
		      pre_rgb = LBL_COR;
		      if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[I_COR]++;
		    }
		    else
		    {
		      pre_rgb = LBL_EDGE;
		      if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[I_EDGE]++;
		    }
		    // special case:  combined class for corner and edge
		    if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE) stats.fp[I_EDGECORNER]++;
		  }
		  else
		  {
		    if (c_max < c_r1 * c_min)
		    {
		      pre_rgb = LBL_SPH;
		      if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[I_SPH]++;
		    }
		    else
		    {
		      pre_rgb = LBL_CYL;
		      if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[I_CYL]++;
		    }
		    // special case:  combined class for sphere and cylinder
		    if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL) stats.fp[I_CURVED]++;
		  }

		  switch(exp_rgb) 
		  {
		  case LBL_PLANE:
		    if (pre_rgb != exp_rgb) stats.fn[I_PLANE]++;
		    stats.exp[I_PLANE]++;
		    break;
		  case LBL_EDGE:
		    if (pre_rgb != exp_rgb) 
		    {
		      stats.fn[I_EDGE]++;
		      if (pre_rgb != LBL_COR) stats.fn[I_EDGECORNER]++;
		    }
		    stats.exp[I_EDGE]++;
		    stats.exp[I_EDGECORNER]++;
		    break;
		  case LBL_COR:
		    if (pre_rgb != exp_rgb)
		    {
		      stats.fn[I_COR]++;
		      if (pre_rgb != LBL_EDGE) stats.fn[I_EDGECORNER]++;
		    }
		    stats.exp[I_COR]++;
		    stats.exp[I_EDGECORNER]++;
		    break;
		  case LBL_SPH:
		    if (pre_rgb != exp_rgb)
		    {
		      stats.fn[I_SPH]++;
		      if (pre_rgb != LBL_CYL) stats.fn[I_CURVED]++;
		    }
		    stats.exp[I_SPH]++;
		    stats.exp[I_CURVED]++;
		    break;
		  case LBL_CYL:
		    if (pre_rgb != exp_rgb) 
		    {
		      stats.fn[I_CYL]++;
		      if (pre_rgb != LBL_SPH) stats.fn[I_CURVED]++;
		    }
		    stats.exp[I_CYL]++;
		    stats.exp[I_CURVED]++;
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
      string logfile = folder_ + "results/" + scenes_[i] + "/res_rsd_" + createTimestamp() + ".csv";
      fstream f;
      f.open(logfile.c_str(), fstream::out);
      f << "rn;"<<"rf;"<<"mls;"<<"r_low;"<<"r_high;"<<"r_r1(cyl/sph);"<<"r_r2(cor/edge);"
	<< "all;"<<"undef;"
	<< "exp(Plane);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Edge);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Sphere);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Cylinder);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Corner);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(EdgeCorner);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
	<< "exp(Curved);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"<< endl;
      f.close();

      for (size_t j = 0; j < pc_pcds.size(); j++)
      {
	bool is_mls;
	string rn, rf;
	parseFileName(rsd_pcds[j], rn, rf, is_mls);
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
		SceneResults stats(rn,rf,is_mls);
		for (size_t idx = 0; idx < p->points.size(); idx++)
		{
		  exp_rgb = *reinterpret_cast<int*>(&p->points[idx].rgb); // expected label
		  r_max = rsd->points[idx].r_max;
		  r_min = rsd->points[idx].r_min;
		  
		  if ( r_min > r_high )
		  {
		    pre_rgb = LBL_PLANE;
		    if (exp_rgb != LBL_PLANE && exp_rgb != LBL_UNDEF) stats.fp[I_PLANE]++;
		  }
		  else if (r_min < r_low)
		  {
		    if (r_max < r_r2 * r_min)
		    {
		      pre_rgb = LBL_COR;
		      if (exp_rgb != LBL_COR && exp_rgb != LBL_UNDEF) stats.fp[I_COR]++;
		    }
		    else
		    {
		      pre_rgb = LBL_EDGE;
		      if (exp_rgb != LBL_EDGE && exp_rgb != LBL_UNDEF) stats.fp[I_EDGE]++;
		    }
		    // special case:  combined class for corner and edge
		    if (exp_rgb != LBL_COR && exp_rgb != LBL_EDGE) stats.fp[I_EDGECORNER]++;
		  }
		  else
		  {
		    if (r_max < r_r1 * r_min)
		    {
		      pre_rgb = LBL_SPH;
		      if (exp_rgb != LBL_SPH && exp_rgb != LBL_UNDEF) stats.fp[I_SPH]++;
		    }
		    else
		    {
		      pre_rgb = LBL_CYL;
		      if (exp_rgb != LBL_CYL && exp_rgb != LBL_UNDEF) stats.fp[I_CYL]++;
		    }
		    // special case:  combined class for sphere and cylinder
		    if (exp_rgb != LBL_SPH && exp_rgb != LBL_CYL) stats.fp[I_CURVED]++;
		  }

		  switch(exp_rgb) 
		  {
		  case LBL_PLANE:
		    if (pre_rgb != exp_rgb) stats.fn[I_PLANE]++;
		    stats.exp[I_PLANE]++;
		    break;
		  case LBL_EDGE:
		    if (pre_rgb != exp_rgb) 
		    {
		      stats.fn[I_EDGE]++;
		      if (pre_rgb != LBL_COR) stats.fn[I_EDGECORNER]++;
		    }
		    stats.exp[I_EDGE]++;
		    stats.exp[I_EDGECORNER]++;
		    break;
		  case LBL_COR:
		    if (pre_rgb != exp_rgb)
		    {
		      stats.fn[I_COR]++;
		      if (pre_rgb != LBL_EDGE) stats.fn[I_EDGECORNER]++;
		    }
		    stats.exp[I_COR]++;
		    stats.exp[I_EDGECORNER]++;
		    break;
		  case LBL_SPH:
		    if (pre_rgb != exp_rgb)
		    {
		      stats.fn[I_SPH]++;
		      if (pre_rgb != LBL_CYL) stats.fn[I_CURVED]++;
		    }
		    stats.exp[I_SPH]++;
		    stats.exp[I_CURVED]++;
		    break;
		  case LBL_CYL:
		    if (pre_rgb != exp_rgb) 
		    {
		      stats.fn[I_CYL]++;
		      if (pre_rgb != LBL_SPH) stats.fn[I_CURVED]++;
		    }
		    stats.exp[I_CYL]++;
		    stats.exp[I_CURVED]++;
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

    if (en_fpfh)
    {

    }
  }

  return 0;
}
