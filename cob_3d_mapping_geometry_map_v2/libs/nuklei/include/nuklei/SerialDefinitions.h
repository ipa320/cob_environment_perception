// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_SERIALDEFINITIONS_H
#define NUKLEI_SERIALDEFINITIONS_H

#include <nuklei/Serial.h>
#include <nuklei/BoostSerialization.h>
#include <nuklei/Match.h>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>

namespace nuklei {
  
  const unsigned int bsFlags = 0;
  //static const unsigned int bsFlags = numify<unsigned int>(getenv("BSFLAGS"));
  
  // To compile a serialization stub, `serialize' methods have to be
  // instantiated through the whole class hierarchy, three times (once
  // for each boost archive -- xml, text, bin). This is painfully
  // slow, we no like slow, we do it once here rather than once in
  // each Product.
  
  template<typename T>
  void Serial::readObject(T &object, const std::string& filename,
                          const std::string &typeName)
  {
    NUKLEI_TRACE_BEGIN();
    Type type = typeFromName<Serial>(typeName);
    
    std::auto_ptr<std::istream> ifs;
    if (type == BOOSTXML_COMPRESSED || type == BOOSTBIN_COMPRESSED)
    {
      std::auto_ptr<boost::iostreams::filtering_istream>
      decompressingIfs(new boost::iostreams::filtering_istream);
      decompressingIfs->push(boost::iostreams::gzip_decompressor());
      decompressingIfs->push(boost::iostreams::file_source(filename));
      ifs = decompressingIfs;
    }
    else if (type == BOOSTXML || type == BOOSTBIN)
    {
      ifs.reset(new std::ifstream(filename.c_str()));
    }
    else throw SerialError(std::string("Unknown output format `") + typeName);
    
    NUKLEI_ASSERT(ifs->good());
    
    switch (type)
    {
      case BOOSTXML:
      case BOOSTXML_COMPRESSED:
      {
        boost::archive::xml_iarchive ia(*ifs, bsFlags);
        ia & BOOST_SERIALIZATION_NVP(object);
        // This check always fails:
        //if (ifs->peek() != EOF) throw SerialError("EOF not reached");
        break;
      }
      case BOOSTBIN:
      case BOOSTBIN_COMPRESSED:
      {
        boost::archive::binary_iarchive ia(*ifs, bsFlags);
        ia & BOOST_SERIALIZATION_NVP(object);
        if (ifs->peek() != EOF) throw SerialError("EOF not reached");
        break;
      }
      default:
      {
        throw SerialError(std::string("Unknown output format `") + typeName);
        break;
      }
    }
    NUKLEI_TRACE_END();
  }
  
  template<typename T>
  void Serial::writeObject(const T &object, const std::string& filename,
                           const std::string &typeName,
                           const int precision)
  {
    NUKLEI_TRACE_BEGIN();
    Type type = typeFromName<Serial>(typeName);
    
    std::auto_ptr<std::ostream> ofs;
    if (type == BOOSTXML_COMPRESSED || type == BOOSTBIN_COMPRESSED)
    {
      std::auto_ptr<boost::iostreams::filtering_ostream>
      decompressingOfs(new boost::iostreams::filtering_ostream);
      decompressingOfs->push(boost::iostreams::gzip_compressor());
      decompressingOfs->push(boost::iostreams::file_sink(filename));
      ofs = decompressingOfs;
    }
    else if (type == BOOSTXML || type == BOOSTBIN)
    {
      ofs.reset(new std::ofstream(filename.c_str()));
    }
    else throw SerialError(std::string("Unknown output format `") + typeName);
    
    NUKLEI_ASSERT(ofs->good());
    object.assertConsistency();
    
    switch (type)
    {
      case BOOSTXML:
      case BOOSTXML_COMPRESSED:
      {
        boost::archive::xml_oarchive oa(*ofs, bsFlags);
        oa & BOOST_SERIALIZATION_NVP(object);
        break;
      }
      case BOOSTBIN:
      case BOOSTBIN_COMPRESSED:
      {
        boost::archive::binary_oarchive oa(*ofs, bsFlags);
        oa & BOOST_SERIALIZATION_NVP(object);
        break;
      }
      default:
      {
        throw SerialError(std::string("Unknown output format `") + typeName + "'.");
        break;
      }
    }
    NUKLEI_TRACE_END();
  }
  
}

#endif

