/*
 * marker_container.h
 *
 *  Created on: 14.11.2012
 *      Author: josh
 */

#ifndef MARKER_CONTAINER_H_
#define MARKER_CONTAINER_H_

#include <rosbag/bag.h>
#include "marker_list.h"

namespace cob_3d_marker {

  class MarkerCreator {
  public:
    virtual void marker(cob_3d_marker::MarkerContainer &mc) const=0;
  };

  class MarkerWrite {
  public:
    virtual void write(const visualization_msgs::Marker &marker)=0;
  };

  class MarkerPublisher : public MarkerWrite {
    ros::Publisher &pub_;
  public:
    MarkerPublisher(ros::Publisher &pub):pub_(pub) {}

    virtual void write(const visualization_msgs::Marker &marker) {
      pub_.publish(marker);
    }
  };

  class MarkerBagfile : public MarkerWrite {
    rosbag::Bag *bag_;
    std::string topic_;
    ros::Time time_;
  public:
    MarkerBagfile(rosbag::Bag *bag, const std::string topic="/markers", const ros::Time &time = ros::Time::now()):bag_(bag),topic_(topic), time_(time) {}

    virtual void write(const visualization_msgs::Marker &marker) {
      bag_->write(topic_, time_, marker);
    }
  };

  class MarkerClean {};

  class MarkerContainer {
    typedef std::map<int, boost::shared_ptr<MarkerList> > MT;
    MT markers_;
  public:

    void add(MarkerList *m) {markers_[m->getId()].reset(m);}

    boost::shared_ptr<MarkerList> get(const int id) {
      MT::iterator it = markers_.find(id);
      if(it==markers_.end())
        return boost::shared_ptr<MarkerList>();
      return it->second;
    }

    void write(MarkerWrite *mw) {
      for(MT::const_iterator it=markers_.begin(); it!=markers_.end(); it++)
        for(size_t i=0; i<it->second->marker_.size(); i++)
          mw->write(it->second->marker_[i]);
    }

    void clear() {
      markers_.clear();
    }

    MarkerContainer &operator<<(MarkerList *m) {add(m);return *this;}
    MarkerContainer &operator<<(const MarkerCreator &c) {c.marker(*this);return *this;}
    MarkerContainer &operator<<(const MarkerClean &c) {clean();return *this;}

    MarkerContainer &operator>>(MarkerWrite *bag) {write(bag);return *this;}

    void clean() {
      for(MT::const_iterator it=markers_.begin(); it!=markers_.end(); it++)
        for(size_t i=0; i<it->second->marker_.size(); i++)
          it->second->marker_[i].action = visualization_msgs::Marker::DELETE;
    }
  };
}



#endif /* MARKER_CONTAINER_H_ */
