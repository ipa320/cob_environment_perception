// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */


#ifndef NUKLEI_BOOST_SERIALIZATION_H
#define NUKLEI_BOOST_SERIALIZATION_H

#include <boost/config.hpp>

#include <boost/archive/archive_exception.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/optional.hpp>

#include <boost/serialization/split_free.hpp>

// From boost_1_36_0/libs/serialization/example/demo_auto_ptr.cpp
// Modified to support XML archives.
// I don't know why this is not included in the installed headers.

namespace boost { 
  namespace serialization {

    /////////////////////////////////////////////////////////////
    // implement serialization for auto_ptr<T>
    // note: this must be added to the boost namespace in order to
    // be called by the library
    template<class Archive, class T>
    inline void save(
        Archive & ar,
        const std::auto_ptr<T> &t,
        const unsigned int file_version
    ){
        // only the raw pointer has to be saved
        // the ref count is rebuilt automatically on load
        const T * const raw_ptr = t.get();
        ar << BOOST_SERIALIZATION_NVP(raw_ptr);
    }

    template<class Archive, class T>
    inline void load(
        Archive & ar,
        std::auto_ptr<T> &t,
        const unsigned int file_version
    ){
        T *raw_ptr;
        ar >> BOOST_SERIALIZATION_NVP(raw_ptr);
        t.reset(raw_ptr);
    }

    // split non-intrusive serialization function member into separate
    // non intrusive save/load member functions
    template<class Archive, class T>
    inline void serialize(
        Archive & ar,
        std::auto_ptr<T> &t,
        const unsigned int file_version
    ){
        boost::serialization::split_free(ar, t, file_version);
    }

  } // namespace serialization
} // namespace boost


#include <boost/ptr_container/serialize_ptr_vector.hpp>
#include <boost/ptr_container/serialize_ptr_array.hpp>

// Should come after archives.
#include <boost/serialization/export.hpp>
#include <boost/serialization/base_object.hpp>

#endif

