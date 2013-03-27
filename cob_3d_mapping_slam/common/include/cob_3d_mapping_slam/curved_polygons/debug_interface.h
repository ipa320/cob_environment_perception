/*
 * debug_interface.h
 *
 *  Created on: 12.06.2012
 *      Author: josh
 */

#ifndef DEBUG_INTERFACE_H_
#define DEBUG_INTERFACE_H_

#ifdef DEBUG_

namespace Debug
{
  class Interface
  {
    struct ARROW {
      Eigen::Vector3f from, to;
      int r,g,b;
    };

    std::vector<ARROW> arrows_;
    double time_;
  public:
    void addArrow(const Eigen::Vector3f &from, const Eigen::Vector3f &to, int r=255, int g=255, int b=255)
    {
      ARROW A;A.from=from;A.to=to;
      A.r=r;A.g=g;A.b=b;
      arrows_.push_back(A);
    }

    bool getArrow(Eigen::Vector3f &from, Eigen::Vector3f &to, unsigned char &r, unsigned char &g, unsigned char &b)
    {
      if(arrows_.empty())
        return false;
      from = arrows_.back().from;
      to = arrows_.back().to;
      r = arrows_.back().r;
      g = arrows_.back().g;
      b = arrows_.back().b;
      arrows_.pop_back();
      return true;
    }

    void clear()
    {
      arrows_.clear();
    }

    void setTime(const double t) {time_ = t;}
    double getTime() const {return time_;}
    void sayTook(const char *str) const {
      std::cout<<"took "<<str<<": "<<(ros::Time::now().toSec()-getTime())<<"\n";
    }

    static Interface &get() {
      static Interface intf;
      return intf;
    }
  };
}
#endif

#endif /* DEBUG_INTERFACE_H_ */
