// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */


#ifndef NUKLEI_PLOTTER_H
#define NUKLEI_PLOTTER_H

#include <map>
#include <set>
#include <vector>
#include <cassert>
#include <fstream>
#include <nuklei/Common.h>

namespace nuklei {
  
#define NUKLEI_PLOTTER_F(p, x, y) p.push(#y, x, y)
#define NUKLEI_PLOTTER_D(p, x) p.push(#x, x)
#define NUKLEI_PLOTTER_COMMENT(p, var) p.comment(#var, var)
  
  class Plotter
  {
  public:
    typedef double T;
    typedef std::string mapkey_t;
    typedef std::vector<std::string> comment_collection;
    typedef std::vector< std::pair<T,T> > function_t;
    typedef std::vector<T> density_t;
    typedef std::map< mapkey_t, function_t > function_map;
    typedef std::map< mapkey_t, density_t > density_map;
    
    void push(mapkey_t key, T val)
    {
      densities_[key].push_back(val);
    }
    
    void push(mapkey_t key, T x, T y)
    {
      functions_[key].push_back(std::make_pair(x, y));
    }
    
    template<typename U>
    void comment(std::string key, U value)
    {
      comments_.push_back(key + " = " +  stringify(value));
    }
    
    void comment(std::string c)
    {
      comments_.push_back(c);
    }
    
    void write_r(std::string filename)
    {
      std::ofstream ofs(filename.c_str(), std::ios::out);
      
      for (std::vector<std::string>::const_iterator v_i = comments_.begin();
           v_i != comments_.end(); v_i++)
        ofs << "# " << *v_i << std::endl;
      ofs << std::endl;
      
      for (function_map::const_iterator fm_i = functions_.begin();
           fm_i != functions_.end(); fm_i++)
      {
        ofs << clean(fm_i->first) << "_x = c(";
        for (function_t::const_iterator f_i = fm_i->second.begin();
             f_i != fm_i->second.end(); f_i++)
        {
          ofs << f_i->first;
          if (f_i != --fm_i->second.end()) ofs << ", ";
        }
        ofs << ");\n" << std::endl;
        
        ofs << clean(fm_i->first) << "_y = c(";
        for (function_t::const_iterator f_i = fm_i->second.begin();
             f_i != fm_i->second.end(); f_i++)
        {
          ofs << f_i->second;
          if (f_i != --fm_i->second.end()) ofs << ", ";
        }
        ofs << ");\n" << std::endl;
      }
      
      for (density_map::const_iterator dm_i = densities_.begin();
           dm_i != densities_.end(); dm_i++)
      {
        ofs << clean(dm_i->first) << " = c( ";
        for (density_t::const_iterator d_i = dm_i->second.begin();
             d_i != dm_i->second.end(); d_i++)
        {
          ofs << *d_i;
          if (d_i != --dm_i->second.end()) ofs << ", ";
        }
        ofs << ");\n" << std::endl;
      }
      
      for (function_map::const_iterator fm_i = functions_.begin();
           fm_i != functions_.end(); fm_i++)
        ofs << "plot(" << clean(fm_i->first) << "_x, "
        << clean(fm_i->first) << "_y"
        << ");\n" << std::endl;
      
      for (density_map::const_iterator dm_i = densities_.begin();
           dm_i != densities_.end(); dm_i++)
        ofs << "plot(density(" << clean(dm_i->first)
        << "));\n" << std::endl;
      
    }
    
    void write_octave(std::string filename)
    {
      std::ofstream ofs(filename.c_str(), std::ios::out);
      
      for (std::vector<std::string>::const_iterator v_i = comments_.begin();
           v_i != comments_.end(); v_i++)
        ofs << "% " << *v_i << std::endl;
      ofs << std::endl;
      
      ofs << "figure; hold on;\n" << std::endl;
      
      for (function_map::const_iterator fm_i = functions_.begin();
           fm_i != functions_.end(); fm_i++)
      {
        ofs << clean(fm_i->first) << " = [ ";
        
        for (function_t::const_iterator f_i = fm_i->second.begin();
             f_i != fm_i->second.end(); f_i++)
        {
          ofs << f_i->first << " " << f_i->second;
          if (f_i != --fm_i->second.end()) ofs << " ; ";
        }
        ofs << "];\n" << std::endl;
      }
      
      
      for (function_map::const_iterator fm_i = functions_.begin();
           fm_i != functions_.end(); fm_i++)
        ofs << "plot(" << clean(fm_i->first) << "(:,1), "
        << clean(fm_i->first) << "(:,2)"
        << ", '-;" << fm_i->first << ";');\n" << std::endl;
      
      if (!densities_.empty())
        ofs << "error(\"KDE not supported in octave.\")\n" << std::endl;
    }
    
    static int main( int argc, char ** argv )
    {
      Plotter p;
      
      p.comment("This is a test.");
      
      const double STEP = .1;
      
      for (double d = -3; d <= 3; d += STEP)
      {
        p.push("y", d, d*d);
        p.push("p", d*d);
      }
      
      p.comment("step", STEP);
      
      p.write_octave("/tmp/octave.m");
      p.write_r("/tmp/r.r");
      
      return 0;
    }
    
  private:
    function_map functions_;
    density_map densities_;
    comment_collection comments_;
    
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
      return "a" + c;
    }
  };
  
  
}

#endif

