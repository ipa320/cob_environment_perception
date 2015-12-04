// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_STOPWATCH_H
#define NUKLEI_STOPWATCH_H

#include <string>
#include <ctime>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <nuklei/Common.h>

namespace nuklei {
  
  class Stopwatch
  {
  public:
    
    typedef enum { QUIET, OCTAVE, TEXT } OutputType;
    
    Stopwatch(const std::string &msg) : msg_(msg), outputType_(TEXT)
    {
      start_ = clock();
      lapstart_ = start_;
      
      struct timeval time;
      if (gettimeofday(&time, NULL) != 0)
        NUKLEI_THROW("Error: gettimeofday.");
      realStart_ = time.tv_sec * 1000000 + time.tv_usec;
      realLapStart_ = realStart_;
    }
    
    std::pair<double, double> split(const std::string &spec)
    {
      return display(start_, realStart_, spec);
    }
    
    std::pair<double, double> split()
    {
      return split("");
    }
    
    std::pair<double, double> lap(const std::string &spec)
    {
      std::pair<double, double> t = display(lapstart_, realLapStart_,
                                            std::string("lap: ") + spec);
      lapstart_ = clock();
      struct timeval time;
      if (gettimeofday(&time, NULL) != 0)
        NUKLEI_THROW("Error: gettimeofday.");
      realLapStart_ = time.tv_sec * 1000000 + time.tv_usec;
      return t;
    }
    
    std::pair<double, double> lap()
    {
      return lap("");
    }
    
    OutputType getOutputType() const
    {
      return outputType_;
    }
    
    void setOutputType(OutputType outputType)
    {
      outputType_ = outputType;
    }
    
    void reset(const std::string &msg)
    {
      msg_ = msg;
      reset();
    }
    
    void reset()
    {
      start_ = clock();
      lapstart_ = start_;
      
      struct timeval time;
      if (gettimeofday(&time, NULL) != 0)
        NUKLEI_THROW("Error: gettimeofday.");
      realStart_ = time.tv_sec * 1000000 + time.tv_usec;
      realLapStart_ = realStart_;
    }
    
  private:
    clock_t start_;
    clock_t lapstart_;
    unsigned long realStart_;
    unsigned long realLapStart_;
    std::string msg_;
    OutputType outputType_;
    
    std::pair<double, double> display(clock_t origin, unsigned long realOrigin,
                                      const std::string &spec)
    {
      clock_t end = clock();
      struct timeval time;
      if (gettimeofday(&time, NULL) != 0)
        NUKLEI_THROW("Error: gettimeofday.");
      unsigned long realEnd = time.tv_sec * 1000000 + time.tv_usec;
      
      std::string s = " ";
      std::string tag = "\033[1;31m<Stopwatch>\033[0m ";
      if (!spec.empty())
      {
        s = std::string(" [") + spec + std::string("]: ");
        tag = "\033[31m<Stopwatch>\033[0m ";
      }
      double seconds = ((double) (end - origin)) / CLOCKS_PER_SEC;
      double realSeconds = double(realEnd - realOrigin) / 1000000;
      
      switch (outputType_)
      {
        case QUIET: break;
        case OCTAVE:
          std::cerr << clean(spec) << " = [ " << clean(spec) << " " <<
          (double) (end - origin) << " ];\n";
          break;
        case TEXT:
          std::cout << tag
          << msg_ << s
          << "cpu=" << seconds << "s "
          << "real=" << realSeconds
          << std::endl;
          break;
      }
      return std::make_pair(seconds, realSeconds);
    }
    
    std::string clean(const std::string &dirty)
    {
      std::string c = dirty;
      for (std::string::iterator s_i = c.begin(); s_i != c.end(); s_i++)
      {
        if (! (*s_i >= 'a' && *s_i <= 'z') &&
            ! (*s_i >= 'A' && *s_i <= 'z') &&
            ! (*s_i >= '0' && *s_i <= '9'))
          *s_i = '_';
      }
      return c;
    }
    
  };
  
}

#endif

