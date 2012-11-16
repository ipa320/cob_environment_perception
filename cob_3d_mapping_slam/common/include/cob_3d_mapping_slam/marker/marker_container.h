/*
 * marker_container.h
 *
 *  Created on: 14.11.2012
 *      Author: josh
 */

#ifndef MARKER_CONTAINER_H_
#define MARKER_CONTAINER_H_

#include "marker_list.h"

namespace cob_3d_marker {

  class MarkerCreator {
  public:
    virtual void marker(cob_3d_marker::MarkerContainer &mc)=0;
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
  public:
    MarkerBagfile(rosbag::Bag *bag, const std::string topic="/markers"):bag_(bag),topic_(topic) {}

    virtual void write(const visualization_msgs::Marker &marker) {
      bag_->write(topic_, ros::Time::now(), marker);
    }
  };

  class MarkerContainer {
    typedef std::map<int, boost::shared_ptr<MarkerList> > MT;
    MT markers_;
  public:

    void add(MarkerList *m) {markers_[m->getId()].reset(m);}

    boost::shared_ptr<MarkerList> get(const int id) {return markers_[id];}

    void write(MarkerWrite *mw) {
      for(MT::const_iterator it=markers_.begin(); it!=markers_.end(); it++)
        for(size_t i=0; i<it->second->marker_.size(); i++)
          mw->write(it->second->marker_[i]);
    }

    void clear() {
      markers_.clear();
    }

    MarkerContainer &operator<<(MarkerList *m) {add(m);return *this;}
    MarkerContainer &operator<<(MarkerCreator &c) {c.marker(*this);return *this;}

    MarkerContainer &operator>>(MarkerWrite *bag) {write(bag);return *this;}

  };
}



#endif /* MARKER_CONTAINER_H_ */
