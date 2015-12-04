// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_PCDOBSERVATION_H
#define NUKLEI_PCDOBSERVATION_H


#include <vector>
#include <string>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <nuklei/Definitions.h>
#include <nuklei/Color.h>
#include <nuklei/LinearAlgebra.h>
#include <nuklei/SerializedKernelObservation.h>

// See http://sampl.ece.ohio-state.edu/data/3DDB/RID/minolta/

namespace nuklei {

  class PCDObservation : public SerializedKernelObservation
  {
  public:
    
    Type type() const { return NUKLEI; }
    
    PCDObservation();
    PCDObservation(const kernel::base& k);
    ~PCDObservation() {};
  };

}

#endif
