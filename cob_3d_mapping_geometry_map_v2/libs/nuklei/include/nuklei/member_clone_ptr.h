// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_MEMBER_CLONE_PTR_H
#define NUKLEI_MEMBER_CLONE_PTR_H

#include <memory>
#include <nuklei/Definitions.h>


namespace nuklei {

  template<typename T>
  struct member_clone_ptr
  {
    member_clone_ptr() {}
    member_clone_ptr(const T& element) : ptr_(element.clone()) {}
    member_clone_ptr(const member_clone_ptr<T>& p)
    {
      if (p.isDefined()) ptr_ = p->clone();
    }
    member_clone_ptr<T>& operator=(const member_clone_ptr<T>& p)
    {
      if (p.isDefined()) ptr_ = p->clone();
      else ptr_.reset();
      return *this;
    }
    
    void assertConsistency() const
    {
      NUKLEI_TRACE_BEGIN();
      if (isDefined()) get().assertConsistency();
      NUKLEI_TRACE_END();
    }
    
    T& operator* () { return get(); }
    const T& operator* () const { return get(); }

    T* operator-> () { return &get(); }
    const T* operator-> () const { return &get(); }
    
    bool isDefined() const { return ptr_.get() != NULL; }
      
    T& get()
    { NUKLEI_ASSERT(isDefined()); return *ptr_; }
    const T& get() const
    { NUKLEI_ASSERT(isDefined()); return *ptr_; }

    void set(const T& element) { ptr_ = element.clone(); }

    void clear() { ptr_.reset(); }

  private:
    std::auto_ptr<T> ptr_;

    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & BOOST_SERIALIZATION_NVP(ptr_);
      }
  };

}

#endif

