// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_LOG_H
#define NUKLEI_LOG_H

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cassert>
#include <boost/thread/mutex.hpp>

namespace nuklei {
  
  class Log
  {
  public:
    static std::ostream& stream()
    {
      assert(out != NULL);
      return *out;
    }
    
    static void setOutput(const std::string &filename);
    static void setOutput(std::ostream *stream);
    
    static std::string breakLines(const std::string &s);
    
    static void log(const char* file,
                    int line,
                    unsigned level,
                    const std::string &s);
    
    
    typedef enum { NEVER = 0, FATAL, ERROR, WARN, INFO,
      LOG, DEBUG, UNKNOWN } Type;
    static const Type defaultType = LOG;
    static const std::string TypeNames[];
    
    //This one should be moved elsewhere sometime.
    static boost::mutex mutex_;
  private:
    static std::string msgColor;
    static std::string errorColor;
    static std::string nocolor;
    
    static std::ostream *out;
    static std::ostream *outInstance;
  };
  
}

#endif
