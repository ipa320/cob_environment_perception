// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_NULLABLE_H
#define NUKLEI_NULLABLE_H

#include <nuklei/Definitions.h>

//+//////////////////////////////////////////////////////////////////////////+//
//+//////// Obsolete -- use boost::optional and boost::none instead /////////+//
//+//////////////////////////////////////////////////////////////////////////+//


namespace nuklei {

  struct undefined {};

  /**
   * @brief Obsolete -- use boost::optional and boost::none instead
   */
  template<typename T>
  struct nullable
  {
    typedef std::pair<bool, T> container_t;

    nullable() : container_(false, T()) {}
    nullable(const undefined&) : container_(false, T()) {}
    nullable(const T& element) : container_(true, element) {}
    nullable(const nullable& n) : container_(n.container_) {}
    
    void assertConsistency() const
    {
      NUKLEI_TRACE_BEGIN();
      if (isDefined()) get().assertConsistency();
      NUKLEI_TRACE_END();
    }
    
    nullable& operator=(const T& element)
    { set(element); return *this; }

    // These are neat, but may make things hard to read.
    // Operator * is good enough.
    //operator T& () { return get(); }
    //operator const T& () const { return get(); }

    T& operator* () { return get(); }
    const T& operator* () const { return get(); }

    T* operator-> () { return &get(); }
    const T* operator-> () const { return &get(); }
    
    bool isDefined() const { return container_.first; }
    
    // The following should work, even for POD types (see 8.5).
    // However this doesn't seem like common practice, and I'm not
    // sure if it will behave the same in all compilers...
    //void define() { set(T()); } 
  
    T& get()
    { NUKLEI_ASSERT(container_.first); return container_.second; }
    const T& get() const
    { NUKLEI_ASSERT(container_.first); return container_.second; }

    void set(const T& element) { container_ = container_t(true, element); }

    void clear() { container_ = container_t(false, T()); }

  private:
    container_t container_;

    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & BOOST_SERIALIZATION_NVP(container_);
      }
  };

}

#endif

