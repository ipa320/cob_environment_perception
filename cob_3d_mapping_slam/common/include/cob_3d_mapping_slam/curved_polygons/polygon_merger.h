/*
 * polygon_merger.h
 *
 *  Created on: 04.07.2012
 *      Author: josh
 */

#ifndef POLYGON_MERGER_H_
#define POLYGON_MERGER_H_

#include <vector>

namespace Slam_CurvedPolygon
{

  class PolygonData {
    std::vector<float> data_;
  public:

    void add(const float x, const float y) {data_.push_back(x); data_.push_back(y);}

    bool get(float &x, float &y) {
      if(data_.size()<2) return false;
      y = data_.back();data_.pop_back();
      x = data_.back();data_.pop_back();
      return true;
    }

    std::vector<float> &get() {return data_;}
    const std::vector<float> &get() const {return data_;}

    void clear() {data_.clear();}
  };

  void mergePolygons(const PolygonData &a, const PolygonData &b, PolygonData &out);
  float unionPolygons(const PolygonData &a, const PolygonData &b, PolygonData &out, bool *sw);
  float area(const PolygonData &a);
}


#endif /* POLYGON_MERGER_H_ */
