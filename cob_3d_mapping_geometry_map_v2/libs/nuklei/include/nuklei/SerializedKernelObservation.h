// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_KERNELOBSERVATION_H
#define NUKLEI_KERNELOBSERVATION_H


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
#include <nuklei/member_clone_ptr.h>


namespace nuklei {

  class SerializedKernelObservation : public Observation
    {
    public:

      Type type() const { return SERIAL; }

      static const double TOL;
 
      std::auto_ptr<kernel::base> getKernel() const
      {
        return k_->clone();
      }
 
      void setKernel(const kernel::base& k)
      {
        NUKLEI_TRACE_BEGIN();
        k_ = k;
        NUKLEI_TRACE_END();
      }

      SerializedKernelObservation();
      SerializedKernelObservation(const kernel::base& k);
      ~SerializedKernelObservation() {};
          
    private:
      member_clone_ptr<kernel::base> k_;
    };

}

#endif
