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
    n.param("/registration/"+name,v,v);
    params_int_[name] = v;
    n.param("/registration/"+name+"_max",v,v);
    params_int_[name+"_max"] = v;
    n.param("/registration/"+name+"_min",v,v);
    params_int_[name+"_min"] = v;
    n.param("/registration/"+name+"_step",v,1);
    params_int_[name+"_step"] = v;

    ParameterBagShortcut<int> r;
    r.name = name;
    r.p = this;
    return r;
  }

  ParameterBagShortcut<double> createParam(ros::NodeHandle &n, const std::string &name, double v) {
    n.param("/registration/"+name,v,v);
    params_double_[name] = v;
    n.param("/registration/"+name+"_max",v,v);
    params_double_[name+"_max"] = v;
    n.param("/registration/"+name+"_min",v,v);
    params_double_[name+"_min"] = v;
    n.param("/registration/"+name+"_step",v,1.);
    params_double_[name+"_step"] = v;

    ParameterBagShortcut<double> r;
    r.name = name;
    r.p = this;
    return r;
  }

  void createParam(ros::NodeHandle &n, const std::string &name, std::string v) {
    n.param("/registration/"+name,v,v);
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
  ros::NodeHandle &n;
public:
  ParameterBucket(ros::NodeHandle &n):n(n) {}

  bool addParameter(const std::string &name) {
    if(!n.hasParam("/registration/"+name)) return false;

    bool ret=false;
    int i;
    if(n.getParam("/registration/"+name,i)) {
      createParam(n,name,i);
    }

    double f;
    if(n.getParam("/registration/"+name,f)) {
      createParam(n,name,(double)f);
    }

    std::string s;
    if(n.getParam("/registration/"+name,s)) {
      createParam(n,name,s);
    }

    return ret;
  }
};


#endif /* PARAMETERS_BAG_H_ */
