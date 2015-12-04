// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_RIFOBSERVATION_H
#define NUKLEI_RIFOBSERVATION_H


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

// See http://sampl.ece.ohio-state.edu/data/3DDB/RID/minolta/

namespace nuklei {

  class RIFObservation : public Observation
    {
    public:
  
      Type type() const { return RIF; }

      static const double TOL;
 
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

      RIFObservation();
      RIFObservation(const kernel::r3& k);
      ~RIFObservation() {};
    
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
