// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_PLYOBSERVATIONSERIAL_H
#define NUKLEI_PLYOBSERVATIONSERIAL_H


#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/PLYObservation.h>


namespace nuklei {


  class PLYReader : public ObservationReader
    {
    public:
      PLYReader(const std::string &observationFileName);
      ~PLYReader();
  
      Observation::Type type() const { return Observation::PLY; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
      std::ifstream in_;
      int index_;
      int n_;
      std::string observationFileName_;
    };

  class PLYWriter : public ObservationWriter
  {
  public:
    PLYWriter(const std::string &observationFileName);
    ~PLYWriter();
    
    Observation::Type type() const { return Observation::PLY; }
    
    void init();
    void reset();
    
    std::auto_ptr<Observation> templateObservation() const
    { return std::auto_ptr<Observation>(new PLYObservation); }
    
    void writeObservation(const Observation &o);
    void writeBuffer();
    
  private:
    std::string observationFileName_;
    std::vector<Vector3> points_;
  };
  
}

#endif

