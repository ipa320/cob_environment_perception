// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_MAP_H
#define NUKLEI_MAP_H

#include <map>

#include <nuklei/Common.h>
#include <nuklei/Definitions.h>
#include <nuklei/BoostSerialization.h>

namespace nuklei {

  template<typename KeyType, typename ValueType>
  struct simple_map
  {
    typedef std::map<KeyType, ValueType> map_impl;
    typedef typename map_impl::value_type KeyValuePair;

    virtual ~simple_map() {}

    void insert(const KeyType& key, const ValueType &v)
    {
      NUKLEI_TRACE_BEGIN();
      bool newKey = map_.insert(std::make_pair(key, v)).second;
      NUKLEI_ASSERT(newKey);
      NUKLEI_TRACE_END();
    }

    void insert(const KeyType& key)
    {
      NUKLEI_TRACE_BEGIN();
      bool newKey = map_.insert(std::make_pair(key, ValueType())).second;
      NUKLEI_ASSERT(newKey);
      NUKLEI_TRACE_END();
    }

    bool has_key(const KeyType& key) const
    {
      NUKLEI_TRACE_BEGIN();
      typename map_impl::const_iterator i = map_.find(key);
      return i != map_.end();
      NUKLEI_TRACE_END();
    }

    KeyValuePair& find(const KeyType& key)
    {
      NUKLEI_TRACE_BEGIN();
      typename map_impl::iterator i = map_.find(key);
      NUKLEI_ASSERT(i != map_.end());
      return *i;
      NUKLEI_TRACE_END();
    }

    const KeyValuePair& find(const KeyType& key) const
    {
      NUKLEI_TRACE_BEGIN();
      typename map_impl::const_iterator i = map_.find(key);
      NUKLEI_ASSERT(i != map_.end());
      return *i;
      NUKLEI_TRACE_END();
    }

    ValueType& operator[](const KeyType& key)
    {
      NUKLEI_TRACE_BEGIN();
      return find(key).second;
      NUKLEI_TRACE_END();
    }

    const ValueType& operator[](const KeyType& key) const
    {
      NUKLEI_TRACE_BEGIN();
      return find(key).second;
      NUKLEI_TRACE_END();
    }

    void erase(const KeyType& key)
    {
      NUKLEI_TRACE_BEGIN();
      unsigned erased = map_.erase(key);
      NUKLEI_ASSERT(erased == 1);
      NUKLEI_TRACE_END();
    }
    
    void clear()
    {
      map_.clear();
    }

  private:
    map_impl map_;

    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & BOOST_SERIALIZATION_NVP(map_);
      }
  };

}

#endif
