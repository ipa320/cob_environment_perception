// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_OFFOBSERVATIONSERIAL_H
#define NUKLEI_OFFOBSERVATIONSERIAL_H


#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/OffObservation.h>

namespace nuklei {


  class OffReader : public ObservationReader
    {
    public:
      OffReader(const std::string &observationFileName);
      ~OffReader();
  
      Observation::Type type() const { return Observation::OFF; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
      std::ifstream in_;
      std::string observationFileName;
      int count_;
    };

  
  class OffWriter : public ObservationWriter
  {
  public:
    OffWriter(const std::string &observationFileName);
    ~OffWriter();
    
    Observation::Type type() const { return Observation::OFF; }
    
    void init();
    void reset();
    
    std::auto_ptr<Observation> templateObservation() const
    { return std::auto_ptr<Observation>(new OffObservation); }
    
    void writeObservation(const Observation &o);
    void writeBuffer();
    
  private:
    std::string observationFileName_;
    std::vector<Vector3> points_;
  };
  

}

#endif

