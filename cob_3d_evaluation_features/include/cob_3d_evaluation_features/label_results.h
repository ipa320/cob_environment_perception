/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 01/2012
*
* \brief
* Defines some commonly used labels for primitive geometries
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef __COB_3D_MAPPING_COMMON_LABEL_RESULTS_H__
#define __COB_3D_MAPPING_COMMON_LABEL_RESULTS_H__

#include <ostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <string>
#include "cob_3d_mapping_common/label_defines.h"

#define NUM_LBL 7

namespace cob_3d_mapping_common
{
  class LabelResults
  {
    public:
    LabelResults(const std::string &rn,
		 const std::string &rf,
		 const bool mls) : rn_(rn), rf_(rf), mls_(mls), undef(0)
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
	all = exp[EVAL_PLANE]+exp[EVAL_EDGE]+exp[EVAL_SPH]+exp[EVAL_CYL]+exp[EVAL_COR];
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

    std::string writeToFile(const std::string &file,
			    const std::string &p1 = "",
			    const std::string &p2 = "",
			    const std::string &p3 = "",
			    const std::string &p4 = "") const
    {
      std::stringstream f;
      f << rn_ <<";"<< rf_ <<";"<< mls_ <<";";
      if (p1 == "")
	f << all <<";"<< undef <<";";
      else if (p2 == "")
	f << p1 <<";"<< all <<";"<< undef <<";";
      else if (p3 == "")
	f << p1 <<";"<< p2 <<";"<< all <<";"<< undef <<";";
      else if (p4 == "")
	f << p1 <<";"<< p2 <<";"<< p3 <<";"<< all <<";"<< undef <<";";
      else
	f << p1 <<";"<< p2 <<";"<< p3 <<";"<< p4 <<";"<< all <<";"<< undef <<";";

      for(size_t i = 0; i < NUM_LBL; ++i)
      {
	f << exp[i] <<";"<< tp[i] <<";"<< fp[i] <<";"<< fn[i] <<";"
	  << prec[i] <<";"<< rec[i] <<";"<< f1[i] <<";";
      }
      if (file != "")
      {
	std::fstream ffile;
	ffile.open(file.c_str(), std::fstream::out | std::fstream::app);
	ffile << f << std::endl;
	ffile.close();
      }
      return f.str();
    }

    std::string writeToFile(const std::string &file,
			    const float p1 = std::numeric_limits<float>::quiet_NaN(),
			    const float p2 = std::numeric_limits<float>::quiet_NaN(),
			    const float p3 = std::numeric_limits<float>::quiet_NaN(),
			    const float p4 = std::numeric_limits<float>::quiet_NaN()) const
    {
      std::stringstream f;
      f << rn_ <<";"<< rf_ <<";"<< mls_ <<";";
      if (p1 != p1)
	f << all <<";"<< undef <<";";
      else if (p2 != p2)
	f << p1 <<";"<< all <<";"<< undef <<";";
      else if (p3 != p3)
	f << p1 <<";"<< p2 <<";"<< all <<";"<< undef <<";";
      else if (p4 != p4)
	f << p1 <<";"<< p2 <<";"<< p3 <<";"<< all <<";"<< undef <<";";
      else
	f << p1 <<";"<< p2 <<";"<< p3 <<";"<< p4 <<";"<< all <<";"<< undef <<";";

      for(size_t i = 0; i < NUM_LBL; ++i)
      {
	f << exp[i] <<";"<< tp[i] <<";"<< fp[i] <<";"<< fn[i] <<";"
	  << prec[i] <<";"<< rec[i] <<";"<< f1[i] <<";";
      }
      if (file != "")
      {
	std::fstream ffile;
	ffile.open(file.c_str(), std::fstream::out | std::fstream::app);
	ffile << f.str() << std::endl;
	ffile.close();
      }
      return f.str();
    }

    std::string writeToFile(const std::string &file = "") const
    {
      std::stringstream f;
      f << rn_ <<";"<< rf_ <<";"<< mls_ <<";"<< all <<";"<< undef <<";";


      for(size_t i = 0; i < NUM_LBL; ++i)
      {
	f << exp[i] <<";"<< tp[i] <<";"<< fp[i] <<";"<< fn[i] <<";"
	  << prec[i] <<";"<< rec[i] <<";"<< f1[i] <<";";
      }
      if (file != "")
      {
	std::fstream ffile;
	ffile.open(file.c_str(), std::fstream::out | std::fstream::app);
	ffile << f.str() << std::endl;
	ffile.close();
      }
      return f.str();
    }


    friend std::ostream& operator<< (std::ostream &os,const LabelResults &obj);

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
    std::string rn_;
    std::string rf_;
    bool mls_;
  };

  std::ostream& operator<< (std::ostream &os, LabelResults &obj)
  {
    obj.calcResults();
    os << obj.writeToFile();
    return os;
  }

  std::string writeHeader(const std::string &file = "",
			  const std::string &p1 = "",
			  const std::string &p2 = "",
			  const std::string &p3 = "",
			  const std::string &p4 = "")
  {
    std::stringstream f;
    f << "rn;"<<"rf;"<<"mls;";
    if (p1 != "")
    {
      if (p2 == "")
	f << p1 <<";";
      else if (p3 == "")
	f << p1 <<";" << p2 <<";";
      else if (p4 == "")
	f << p1 <<";" << p2 <<";" << p3 <<";";
      else
	f << p1 <<";" << p2 <<";" << p3 <<";" << p4 <<";";
    }

    f << "all;"<<"undef;"
      << "exp(Plane);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
      << "exp(Edge);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
      << "exp(Sphere);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
      << "exp(Cylinder);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
      << "exp(Corner);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
      << "exp(EdgeCorner);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;"
      << "exp(Curved);"<<"TP;"<<"FP;"<<"FN;"<<"Prec.;"<<"Rec.;"<<"F1;";

    if (file != "")
    {
      std::fstream ffile;
      ffile.open(file.c_str(), std::fstream::out);
      ffile << f.str() << std::endl;
      ffile.close();
    }
    return f.str();
  }

  std::string createTimestamp()
  {
    struct tm *t;
    time_t rawtime;
    time(&rawtime);
    t = localtime(&rawtime);
    std::stringstream stamp;
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

  void parseFileName(const std::string &file,
		     std::string &rn,
		     std::string &rf,
		     bool &is_mls)
  {
    std::string tmp = "";
    if (std::string::npos != file.rfind("rnmls_"))
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
}

#endif
