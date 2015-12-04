// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_NUKLEI_OBSERVATIONSERIAL_H
#define NUKLEI_NUKLEI_OBSERVATIONSERIAL_H


#include <boost/ptr_container/ptr_list.hpp>
#include <nuklei/Definitions.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/NukleiObservation.h>

#ifdef NUKLEI_USE_TICPP
namespace ticpp {
  class Document;
  class Node;
  class Element;
  template < class T > class Iterator;
}
#else
namespace tinyxml2 {
  class XMLDocument;
  class XMLElement;
}
#endif

namespace nuklei {

  class NukleiReader : public ObservationReader
    {
    public:
      NukleiReader(const std::string &observationFileName);
      ~NukleiReader();
  
      Observation::Type type() const { return Observation::NUKLEI; }

      void reset();
  
    protected:
      void init_();
      std::auto_ptr<Observation> readObservation_();
      std::string observationFileName_;
    private:
#ifdef NUKLEI_USE_TICPP
      boost::shared_ptr<ticpp::Document> in_;
      typedef ticpp::Iterator< ticpp::Element > ElementIterator;
      boost::shared_ptr<ElementIterator> e_;
#else
      boost::shared_ptr<tinyxml2::XMLDocument> in_;
      tinyxml2::XMLElement* e_;
#endif
    };

  class NukleiWriter : public ObservationWriter
    {
    public:
      NukleiWriter(const std::string &observationFileName);
      ~NukleiWriter();
  
      Observation::Type type() const { return Observation::NUKLEI; }

      void init();
      void reset();
      
      std::auto_ptr<Observation> templateObservation() const
      { return std::auto_ptr<Observation>(new NukleiObservation); }

      void writeObservation(const Observation &o);
      void writeBuffer();
      
    private:
      std::string observationFileName_;
#ifdef NUKLEI_USE_TICPP
      boost::shared_ptr<ticpp::Document> out_;
      ticpp::Element* kc_;
#else
      boost::shared_ptr<tinyxml2::XMLDocument> out_;
      tinyxml2::XMLElement* kc_;
#endif
      coord_t totalWeight_;
    };

}

#endif

