// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_DECORATION_H
#define NUKLEI_DECORATION_H


#include <boost/any.hpp>
#include <nuklei/simple_map.h>
#include <nuklei/Definitions.h>

namespace nuklei {
  
  template<typename KeyType>
  struct decoration
  {
    typedef simple_map<KeyType, boost::any> map_t;
    
    template<typename ValueType>
    void insert(const KeyType& key, const ValueType &v)
    {
      NUKLEI_TRACE_BEGIN();
      decorations_.insert(key, v);
      NUKLEI_TRACE_END();
    }
    
    // This is not possible: we need to tell what goes inside the
    // underlying boost::any.
    //void insert(const KeyType& key)
    
    bool has_key(const KeyType& key) const
    {
      NUKLEI_TRACE_BEGIN();
      return decorations_.has_key(key);
      NUKLEI_TRACE_END();
    }
    
    template<typename ValueType>
    ValueType& get(const KeyType& key)
    {
      NUKLEI_TRACE_BEGIN();
      return boost::any_cast<ValueType&>(decorations_[key]);
      NUKLEI_TRACE_END();
    }
    
    template<typename ValueType>
    const ValueType& get(const KeyType& key) const
    {
      NUKLEI_TRACE_BEGIN();
      return boost::any_cast<const ValueType&>(decorations_[key]);
      NUKLEI_TRACE_END();
    }
    
    void erase(const KeyType& key)
    {
      NUKLEI_TRACE_BEGIN();
      decorations_.erase(key);
      NUKLEI_TRACE_END();
    }
    
    void clear()
    {
      decorations_.clear();
    }
    
  private:
    map_t decorations_;
  };
  
}

#endif

