// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_COVIS3DOBSERVATIONSERIAL_H
#define NUKLEI_COVIS3DOBSERVATIONSERIAL_H


#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/CoViS3DObservation.h>

#ifdef NUKLEI_USE_TICPP
namespace ticpp {
  class Document;
  class Node;
  class Element;
  template < class T > class Iterator;
}
#endif

namespace nuklei {

  class CoViS3DReader : public ObservationReader
    {
    public:
      CoViS3DReader(const std::string &observationFileName);
      ~CoViS3DReader();
  
      Observation::Type type() const { return Observation::COVIS3D; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
      std::string observationFileName;
    private:
      boost::shared_ptr<CoViS3DReader> reader_;
    };

  class CoViS3DXMLReader : public CoViS3DReader
    {
    public:
      CoViS3DXMLReader(const std::string &observationFileName);
      ~CoViS3DXMLReader();
  
      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
#ifdef NUKLEI_USE_TICPP
      boost::shared_ptr<ticpp::Document> in_;
      typedef ticpp::Iterator< ticpp::Element > ElementIterator;
      boost::shared_ptr<ElementIterator> e_;
#endif
    };


  class CoViS3DWandererReader : public CoViS3DReader
    {
    public:
      CoViS3DWandererReader(const std::string &observationFileName);
      ~CoViS3DWandererReader();
  
      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
    private:
      std::ifstream in_;
    };

  class CoViS3DXMLWriter : public ObservationWriter
    {
    public:
      CoViS3DXMLWriter(const std::string &observationFileName);
      ~CoViS3DXMLWriter();
  
      Observation::Type type() const { return Observation::COVIS3D; }

      void init();
      void reset();

      std::auto_ptr<Observation> templateObservation() const
      { return std::auto_ptr<Observation>(new CoViS3DObservation); }

      void writeObservation(const Observation &o);
      void writeBuffer();
      
    private:
      std::string observationFileName;
#ifdef NUKLEI_USE_TICPP
      boost::shared_ptr<ticpp::Document> out_;
      ticpp::Element* scene_;
#endif
    };

}

#endif

