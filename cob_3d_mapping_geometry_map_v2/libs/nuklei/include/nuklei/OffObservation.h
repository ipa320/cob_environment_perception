// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_OFFOBSERVATION_H
#define NUKLEI_OFFOBSERVATION_H


#include <vector>
#include <string>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <nuklei/Definitions.h>
#include <nuklei/Color.h>
#include <nuklei/LinearAlgebra.h>
#include <nuklei/Observation.h>

namespace nuklei {

  class OffObservation : public Observation
    {
    public:
      static const double TOL;
      
      Type type() const { return OFF; }
 
      std::auto_ptr<kernel::base> getKernel() const
      {
        return k_.clone();
      }
      
      void setKernel(const kernel::base& k)
      {
        NUKLEI_TRACE_BEGIN();
        k_ = dynamic_cast<const kernel::r3&>(k);
        NUKLEI_TRACE_END();
      }
  
      OffObservation();
      OffObservation(const kernel::r3& k);
      ~OffObservation() {};
    
      void setLoc(Vector3 loc);
      Vector3 getLoc() const;
      
      void setWeight(weight_t weight);
      weight_t getWeight() const;
  
      void setColor(const Color& color);
      const Color& getColor() const;

    private:
      kernel::r3 k_;
    };

}

#endif
