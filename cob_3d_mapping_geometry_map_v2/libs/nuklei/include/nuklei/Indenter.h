// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_INDENTER_H
#define NUKLEI_INDENTER_H

#include <string>
#include <iostream>

namespace nuklei {
  
  
  class Indenter
    {
    public:
      Indenter(std::ostream &out = std::cout, std::string step = "  ");
      ~Indenter();
  
      template<class T>
        friend std::ostream& operator<<(Indenter &idt, const T &t);
  
      static int main(int argc, char ** argv);
  
    private:
      static int level;
      std::ostream &out;
      std::string step;
    };

  template<class T>
    std::ostream& operator<<(Indenter &idt, const T &t)
    {
      std::string tab = "";
      for (int i=0; i < idt.level; i++)
        tab += idt.step;
    
      idt.out << tab << t;
      return idt.out;
    }      

}

#endif
