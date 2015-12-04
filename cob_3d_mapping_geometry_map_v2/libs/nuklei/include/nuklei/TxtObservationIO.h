// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_TXTOBSERVATIONSERIAL_H
#define NUKLEI_TXTOBSERVATIONSERIAL_H


#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/TxtObservation.h>
#include <nuklei/KernelCollection.h>

namespace nuklei {


  class TxtReader : public ObservationReader
    {
    public:
      TxtReader(const std::string &observationFileName);
      ~TxtReader();
  
      Observation::Type type() const { return Observation::TXT; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
      std::ifstream in_;
      std::string observationFileName;
    };

  
  class TxtWriter : public ObservationWriter
  {
  public:
    TxtWriter(const std::string &observationFileName);
    ~TxtWriter();
    
    Observation::Type type() const { return Observation::TXT; }
    
    void init();
    void reset();
    
    std::auto_ptr<Observation> templateObservation() const
    { return std::auto_ptr<Observation>(new TxtObservation); }
    
    void writeObservation(const Observation &o);
    void writeBuffer();
    
  private:
    std::string observationFileName_;
    KernelCollection points_;
  };
  

}

#endif

