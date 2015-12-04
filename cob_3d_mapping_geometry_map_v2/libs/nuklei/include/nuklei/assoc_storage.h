// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_ASSOC_STORAGE_H
#define NUKLEI_ASSOC_STORAGE_H


#include <nuklei/simple_map.h>
#include <nuklei/Definitions.h>

namespace nuklei {
  
  template<typename T>
  struct assoc_storage
  {
    typedef simple_map<id_t, T> map_t;
    typedef typename map_t::KeyValuePair KeyValuePair;
    
    assoc_storage() : counter_(minKey_) {}
    
    KeyValuePair& new_element(const id_t key)
    {
      NUKLEI_TRACE_BEGIN();
      NUKLEI_ASSERT(key >= minKey_);
      KeyValuePair v(key, T());
      map_.insert(key, T());
      return map_.find(key);
      NUKLEI_TRACE_END();
    }
    
    KeyValuePair& new_element()
    {
      NUKLEI_TRACE_BEGIN();
      return new_element(counter_++);
      NUKLEI_TRACE_END();
    }
    
    bool has_key(const id_t key) const
    {
      NUKLEI_TRACE_BEGIN();
      return map_.has_key(key);
      NUKLEI_TRACE_END();
    }
    
    T& operator[](const id_t key)
    {
      NUKLEI_TRACE_BEGIN();
      return map_[key];
      NUKLEI_TRACE_END();
    }
    
    const T& operator[](const id_t key) const
    {
      NUKLEI_TRACE_BEGIN();
      return map_[key];
      NUKLEI_TRACE_END();
    }
    
    void erase(const id_t key)
    {
      NUKLEI_TRACE_BEGIN();
      map_.erase(key);
      NUKLEI_TRACE_END();
    }
    
    void clear()
    {
      map_.clear();
      counter_ = minKey_;
    }
  private:
    map_t map_;
    id_t counter_;
    static const id_t minKey_ = 1000;
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(map_)
      & BOOST_SERIALIZATION_NVP(counter_);
    }
  };
  
}

#endif

