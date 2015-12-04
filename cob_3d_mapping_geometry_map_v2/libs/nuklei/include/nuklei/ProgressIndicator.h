// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_PROGRESSINDICATOR_H
#define NUKLEI_PROGRESSINDICATOR_H

#include <string>

namespace nuklei {
  
  
  class ProgressIndicator
  {
  public:
    
    static const int spanStartDef, spanEndDef;
    static const std::string messageDef;
    static const std::string backspaceSequence;
    
    
    ProgressIndicator();
    ProgressIndicator(const int spanEnd,
                      const std::string &message = messageDef,
                      const unsigned minLogLevel = 0);
    ProgressIndicator(const int spanStart,
                      const int spanEnd,
                      const std::string &message = messageDef,
                      const unsigned minLogLevel = 0);
    virtual ~ProgressIndicator();

    void initialize(const int spanStart,
                    const int spanEnd,
                    const std::string &message,
                    const unsigned minLogLevel);

    void setMinLogLevel(const unsigned l);
    unsigned getMinLogLevel() const;
    
    void setValue(const int value);
    void setBackspace(const bool backspace);
    void inc(const int value = 1);
    void mtInc(const int value = 1);
    const std::string& getMessage() const;
    void setMessage(const std::string& message);
    void rewind();
    void forceEnd();
    static int main(int argc, char ** argv);
  private:
    bool verbose_, backspace_;
    unsigned minLogLevel_;
    enum { ready, running, finished } state_;
    int spanStart_, spanEnd_, spanLength_, spanPos_, current_, scale_;
    std::string message_, colorStart_, colorEnd_;
    /*static const int spanStartDef, spanEndDef;
     static const char messageDef[];
     static const char backspaceSequence[];*/
  };
  
  
}

#endif

