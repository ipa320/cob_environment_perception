// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_OSUTXTOBSERVATION_H
#define NUKLEI_OSUTXTOBSERVATION_H


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

  class OsuTxtObservation : public Observation
    {
    public:
  
      Type type() const { return OSUTXT; }

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

      OsuTxtObservation();
      OsuTxtObservation(const kernel::r3& k);
      ~OsuTxtObservation() {};
    
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
