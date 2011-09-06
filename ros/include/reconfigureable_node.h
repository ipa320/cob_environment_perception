#pragma once

#include <dynamic_reconfigure/server.h>

template <class T>
class Reconfigurable_Node {
protected:
	Reconfigurable_Node(const std::string& ns) : srv(ros::NodeHandle(getCorrectedNS(ns)))
	{
	}


	void setReconfigureCallback(const boost::function<void(T &, uint32_t level)> &callback) {
	  srv.setCallback(callback);
	}

private:
	dynamic_reconfigure::Server<T> srv;

	std::string getCorrectedNS(const std::string &s) {
	  static int num=0;

	  std::ostringstream oss;
	  oss << "~/" << s << num;
	  ++num;

	  return oss.str();
	}

};
