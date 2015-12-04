// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_CRDOBSERVATIONSERIAL_H
#define NUKLEI_CRDOBSERVATIONSERIAL_H


#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/CrdObservation.h>

namespace nuklei {


  class CrdReader : public ObservationReader
    {
    public:
      CrdReader(const std::string &observationFileName);
      ~CrdReader();
  
      Observation::Type type() const { return Observation::CRD; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
      std::ifstream in_;
      std::string observationFileName;
    };

  
  class CrdWriter : public ObservationWriter
  {
  public:
    CrdWriter(const std::string &observationFileName, bool syncpc = false);
    ~CrdWriter();
    
    Observation::Type type() const { return Observation::CRD; }
    
    void init();
    void reset();
    
    std::auto_ptr<Observation> templateObservation() const
    { return std::auto_ptr<Observation>(new CrdObservation); }
    
    void writeObservation(const Observation &o);
    void writeBuffer();
    
  private:
    std::string observationFileName_;
    bool syncpc_; // write "syncpc" on the first line.
    std::vector<Vector3> points_;
  };
  

}

#endif

