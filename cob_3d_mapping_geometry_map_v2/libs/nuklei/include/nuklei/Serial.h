// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_SERIAL_H
#define NUKLEI_SERIAL_H

#include <string>
#include <list>
#include <nuklei/Common.h>

namespace nuklei {
  
  class SerialError : public Error
  {
  public:
    SerialError(const std::string &s) : Error(s) {}
  };
  
  struct Serial
  {
    
    // The template methods of this class are defined in Serial.cpp.
    // One will then instantiate them as needed in that same file.
    // The reason for this is the dramatically long compilation time,
    // due to the effort in processing the boost serialization library,
    // and also due to the depth of the class hierarchy (each of these
    // functions instantiates the serialize() template methods of almost
    // all classes in the Nuklei library -- from Graph to Location...).
    
    template<typename T>
    static void readObject(T &object, const std::string& filename,
                           const std::string &typeName);
    
    template<typename T>
    static void readObject(T &object, const std::string& filename)
    {
      NUKLEI_TRACE_BEGIN();
      std::string errorsCat = std::string("Error at input for file `") +
      filename + "'\nErrors at each format attempt were:";
      
      // ideally, each block should try and catch
      // boost::archive::archive_exception's.
      
      std::list<std::string> formats;
      
      formats.push_back("bbinc");
      formats.push_back("bxmlc");
      formats.push_back("bbin");
      formats.push_back("bxml");
      
      for (std::list<std::string>::const_iterator i = formats.begin();
           i != formats.end(); ++i)
      {
        try {
          readObject<T>(object, filename, *i);
          return;
        } catch (std::exception &e) {
          errorsCat += "\n" + std::string(e.what());
        }
      }
      
      throw SerialError(errorsCat);
      NUKLEI_TRACE_END();      
    }
    
    template<typename T>
    static void writeObject(const T &object, const std::string& filename,
                            const std::string &typeName = SERIALIZATION_DEFAULT_BOOST_ARCHIVE,
                            const int precision = PRECISION);
    
    typedef enum { BOOSTXML = 0, BOOSTXML_COMPRESSED,
      BOOSTBIN, BOOSTBIN_COMPRESSED,
      UNKNOWN } Type;
    static const Type defaultType = BOOSTBIN_COMPRESSED;
    static const std::string TypeNames[];
  };
  
}

#endif

