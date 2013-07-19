/*
 * id.h
 *
 *  Created on: 29.08.2012
 *      Author: josh
 */

#ifndef ID_H_
#define ID_H_


namespace Slam_CurvedPolygon
{
  /**
   * S_ID contains more than one id
   * different ids can mean same object, but this can only be recognized after registration
   */
  struct S_ID { //TODO: replace with sorted list
    std::vector<int> ids_;

    void operator+=(const int id) {
      if(!(*this==id))
        ids_.push_back(id);
    }

    void operator+=(const S_ID &o) {
      for(size_t i=0; i<o.ids_.size(); i++)
        *this += o.ids_[i];
    }

    bool operator==(const int id) const {
      for(size_t i=0; i<ids_.size(); i++)
        if(ids_[i]==id)
          return true;
      return false;
    }

    bool operator==(const S_ID &o) const {
      for(size_t i=0; i<ids_.size(); i++)
        if(o==ids_[i])
          return true;
      return false;
    }
  };
}

#endif /* ID_H_ */
