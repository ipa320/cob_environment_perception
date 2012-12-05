//TODO: deprecated, can probably be deleted

#pragma once

#include <dynamic_reconfigure/server.h>

template <class T>
class Reconfigurable_Node {
protected:
  ros::ServiceServer set_service_;
  Reconfigurable_Node(const std::string& ns) : _n(ros::NodeHandle(getCorrectedNS(ns))), srv(_n), first_(true)
  {
    update_pub_ = _n.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
  }

  void setReconfigureCallback(const boost::function<void(T &, uint32_t level)> &callback) {
    srv.setCallback(callback);
  }

  void setReconfigureCallback2(const boost::function<void(T &, uint32_t level)> &callback_s, const boost::function<void(T &)> &callback_get) {
    callback_set_=callback_s;
    callback_get_=callback_get;
    srv.setCallback(boost::bind(&callback_set, this, _1, _2));
    //srv.setCallback(callback_s);
  }

  void updateConfig(const T &config_)
  {
    config_.__toServer__(_n);
    dynamic_reconfigure::Config msg;
    config_.__toMessage__(msg);
    update_pub_.publish(msg);
  }

  // callback for dynamic reconfigure
  static void callback_set(Reconfigurable_Node<T> *inst, T &config, uint32_t level)
  {
    if(!inst->first_)
      inst->callback_set_(config, level);
    else {
      inst->callback_get_(config);
      inst->updateConfig(config);
    }
    inst->first_=false;
  }

private:
  ros::NodeHandle _n;
  dynamic_reconfigure::Server<T> srv;
  ros::Publisher update_pub_;
  bool first_;
  boost::function<void(T &, uint32_t level)> callback_set_;
  boost::function<void(T &)> callback_get_;

  std::string getCorrectedNS(const std::string &s) {
    static int num=0;

    std::ostringstream oss;
    oss << "~/" << s << num;
    ++num;

    return oss.str();
  }

};
