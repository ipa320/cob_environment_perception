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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
/*
 * parameters_bag.h
 *
 *  Created on: Nov 11, 2011
 *      Author: goa-jh
 */

#ifndef PARAMETERS_BAG_H_
#define PARAMETERS_BAG_H_

class ParameterBag
{

  std::map<std::string, int> params_int_;
  std::map<std::string, double> params_double_;
  std::map<std::string, std::string> params_string_;

public:

  template <typename T>
  struct ParameterBagShortcut {
    std::string name;
    ParameterBag *p;

    ParameterBagShortcut setMax(T v) {
      if(p->params_int_.find(name)!=p->params_int_.end())
        p->params_int_[name+"_max"] = v;
      if(p->params_double_.find(name)!=p->params_double_.end())
        p->params_double_[name+"_max"] = v;
      return *this;
    }

    ParameterBagShortcut setMin(T v) {
      if(p->params_int_.find(name)!=p->params_int_.end())
        p->params_int_[name+"_min"] = v;
      if(p->params_double_.find(name)!=p->params_double_.end())
        p->params_double_[name+"_min"] = v;
      return *this;
    }

    ParameterBagShortcut setStep(T v) {
      if(p->params_int_.find(name)!=p->params_int_.end())
        p->params_int_[name+"_step"] = v;
      if(p->params_double_.find(name)!=p->params_double_.end())
        p->params_double_[name+"_step"] = v;
      return *this;
    }

  };

  const std::map<std::string, int> &getInts() const {return params_int_;}
  const std::map<std::string, double> &getDoubles() const {return params_double_;}
  const std::map<std::string, std::string> &getStrings() const {return params_string_;}

  ParameterBagShortcut<int> createParam(ros::NodeHandle &n, const std::string &name, int v) {
    n.param("registration_nodelet/"+name,v,v);
    params_int_[name] = v;
    n.param("registration_nodelet/"+name+"_max",v,v);
    params_int_[name+"_max"] = v;
    n.param("registration_nodelet/"+name+"_min",v,v);
    params_int_[name+"_min"] = v;
    n.param("registration_nodelet/"+name+"_step",v,1);
    params_int_[name+"_step"] = v;

    ParameterBagShortcut<int> r;
    r.name = name;
    r.p = this;
    return r;
  }

  ParameterBagShortcut<double> createParam(ros::NodeHandle &n, const std::string &name, double v) {
    n.param("registration_nodelet/"+name,v,v);
    params_double_[name] = v;
    n.param("registration_nodelet/"+name+"_max",v,v);
    params_double_[name+"_max"] = v;
    n.param("registration_nodelet/"+name+"_min",v,v);
    params_double_[name+"_min"] = v;
    n.param("registration_nodelet/"+name+"_step",v,1.);
    params_double_[name+"_step"] = v;

    ParameterBagShortcut<double> r;
    r.name = name;
    r.p = this;
    return r;
  }

  void createParam(ros::NodeHandle &n, const std::string &name, std::string v) {
    n.param("registration_nodelet/"+name,v,v);
    params_string_[name] = v;
  }

  void setParam(const std::string &name, int v) {params_int_[name] = v;}
  void setParam(const std::string &name, const std::string &v) {params_string_[name] = v;}
  void setParam(const std::string &name, double v) {params_double_[name] = v;}

  bool getParam(const std::string &name, int &val) {
    if(params_int_.find(name)==params_int_.end()) return false;
    val=params_int_[name];
    return true;
  }
  bool getParam(const std::string &name, double &val) {
    if(params_double_.find(name)==params_double_.end()) return false;
    val=params_double_[name];
    return true;
  }
  bool getParam(const std::string &name, std::string &val) {
    if(params_string_.find(name)==params_string_.end()) return false;

    val=params_string_[name];
    return true;
  }

  bool getMax(const std::string &name, int &val) {return getParam(name+"_max",val);}
  bool getMax(const std::string &name, double &val) {return getParam(name+"_max",val);}
  bool getMin(const std::string &name, int &val) {return getParam(name+"_min",val);}
  bool getMin(const std::string &name, double &val) {return getParam(name+"_min",val);}
  bool getStep(const std::string &name, int &val) {return getParam(name+"_step",val);}
  bool getStep(const std::string &name, double &val) {return getParam(name+"_step",val);}
};

class ParameterBucket : public ParameterBag
{
  ros::NodeHandle n;
public:
  ParameterBucket(ros::NodeHandle &n):n(n) {}
  ParameterBucket() {}

  void setNodeHandle(ros::NodeHandle &_n) {n=_n;}

  bool addParameter(const std::string &name) {
    if(!n.hasParam("registration_nodelet/"+name)) return false;

    bool ret=false;
    int i;
    if(n.getParam("registration_nodelet/"+name,i)) {
      createParam(n,name,i);
    }

    double f;
    if(n.getParam("/"+name,f)) {
      createParam(n,name,(double)f);
    }

    std::string s;
    if(n.getParam("registration_nodelet/"+name,s)) {
      createParam(n,name,s);
    }

    return ret;
  }
};


#endif /* PARAMETERS_BAG_H_ */
