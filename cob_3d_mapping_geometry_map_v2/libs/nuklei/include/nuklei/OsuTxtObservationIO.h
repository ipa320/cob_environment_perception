// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_OSUTXTOBSERVATIONSERIAL_H
#define NUKLEI_OSUTXTOBSERVATIONSERIAL_H


#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/OsuTxtObservation.h>


namespace nuklei {


  class OsuTxtReader : public ObservationReader
    {
    public:
      OsuTxtReader(const std::string &observationArgs);
      ~OsuTxtReader();
  
      Observation::Type type() const { return Observation::OSUTXT; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
      std::ifstream in_;
      std::string geometryFileName;
      std::string appFileName;
      
      unsigned rows_;
      unsigned columns_;
      unsigned currentIndex_;
      
      std::vector<bool> flags_;
      std::vector<coord_t> x_;
      std::vector<coord_t> y_;
      std::vector<coord_t> z_;
      std::vector<Vector3> rgb_;
    };

}

#endif

