// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_OBSERVATION_H
#define NUKLEI_OBSERVATION_H


#include <nuklei/Definitions.h>
#include <nuklei/Types.h>
#include <nuklei/Kernel.h>

namespace nuklei {

  class Observation
    {
    public:
      typedef enum { SERIAL, NUKLEI, COVIS3D, OSUTXT, PCD, PLY, RIF, CRD, OFF, BUILTINVTK, TXT, IIS, UNKNOWN } Type;
      static const Type defaultType = SERIAL;
      static const std::string TypeNames[];

      virtual ~Observation() {}
      virtual std::auto_ptr<kernel::base> getKernel() const = 0;
      virtual void setKernel(const kernel::base& k) = 0;
      virtual Type type() const = 0;
    };
}

#endif
