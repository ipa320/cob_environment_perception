// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_OBSERVATIONSERIAL_H
#define NUKLEI_OBSERVATIONSERIAL_H

#include <typeinfo>
#include <boost/utility.hpp>

#include <nuklei/Definitions.h>
#include <nuklei/Observation.h>
#include <nuklei/RegionOfInterest.h>
#include <nuklei/nullable.h>

namespace nuklei {
  
  class KernelCollection;
  
  class ObservationIOError : public Error
    {
    public: ObservationIOError(const std::string &s) : Error(s) {}
    };


  /**
   * @brief Base class for kernel reader and point reader classes.
   *
   * This class also provides methods for reading a file with automatic type
   * detection.
   */
  class ObservationReader : boost::noncopyable
    {
    public:
      virtual ~ObservationReader();
      
      std::auto_ptr<Observation> readObservation();
      std::auto_ptr<KernelCollection> readObservations();
      void readObservations(KernelCollection &kc);
  
      virtual Observation::Type type() const = 0;

      void registerType(Observation::Type t)
      {
        oc.type_ = nameFromType<Observation>(t);
      }

      void registerType(const ObservationReader& reader)
      {
        oc.type_ = typeid(reader).name();
      }

      virtual nullable<unsigned> nObservations() const
      { return undefined(); }

      virtual void addRegionOfInterest(boost::shared_ptr<RegionOfInterest> roi);
  

      virtual void init() { registerType(*this); init_(); }
      virtual void reset() = 0;
  
      class Counter
        {
          typedef std::map<std::string, unsigned> map_t;
          typedef std::list<std::string> list_t;
        public:
          Counter() {}
        
          void incLabel(const std::string &label);
          bool empty() const;
        
          //private:
          friend std::ostream& operator<<(std::ostream &out, const Counter &c);
          std::string type_;
          map_t counts_;
          list_t labels_;
        };

      static std::auto_ptr<ObservationReader>
      createReader(const std::string& arg);
      static std::auto_ptr<ObservationReader>
      createReader(const std::string& arg, const Observation::Type t);
    protected:
      virtual std::auto_ptr<Observation> readObservation_() = 0;
      virtual void init_() = 0;
      Counter oc;
    private:
      boost::shared_ptr<RegionOfInterest> roi_;
    };

  std::ostream& operator<<(std::ostream &out, const ObservationReader::Counter &c);

  /**
   * @brief Reads the data available from the reader @p r and stores it into @p
   * kc.
   */
  void readObservations(ObservationReader& r, KernelCollection &kc);
  /**
   * @brief Reads the file @p s (with automatic type detection) and stores the
   * read data into @p kc.
   */
  void readObservations(const std::string &s, KernelCollection &kc);
  /**
   * @brief Reads the file @p s (with automatic type detection) and stores the
   * read data into @p kc.
   *
   * @p t is set to the format of the file @p s.
   */
  void readObservations(const std::string &s, KernelCollection &kc,
                        Observation::Type& t);
  /**
   * @brief Reads the file @p s (@em no automatic type detection) and stores the
   * read data into @p kc.
   *
   * @p t is the format in which the data is stored.
   */
  void readObservationsWithSpecificFormat(const std::string &s,
                                          KernelCollection &kc,
                                          const Observation::Type& t);

  /**
   * @brief Reads a single observation from file @p s (with automatic type
   * detection), and returns it.
   *
   * This method check that the file @p s contains a single observation. If it
   * is not the case, an exception is thrown.
   */
  kernel::base::ptr readSingleObservation(const std::string &s);
  /**
   * @brief Reads a single observation from file @p s (with automatic type
   * detection), and returns it.
   *
   * @p t is set to the format of the file @p s.
   *
   * This method check that the file @p s contains a single observation. If it
   * is not the case, an exception is thrown.
   */
  kernel::base::ptr readSingleObservation(const std::string &s,
                                          Observation::Type& t);
  /**
   * @brief Reads a single observation from file @p s (@em no automatic type
   * detection), and returns it.
   *
   * @p t is the format in which the data is stored.
   *
   * This method check that the file @p s contains a single observation. If it
   * is not the case, an exception is thrown.
   */
  kernel::base::ptr
  readSingleObservationWithSpecificFormat(const std::string &s,
                                          const Observation::Type& t);

  /** @brief Base class for kernel writer and point writer classes. */
  class ObservationWriter : boost::noncopyable
    {
    public:
      virtual ~ObservationWriter();
      
      virtual void writeObservation(const Observation &o) = 0;
      void writeObservations(const KernelCollection &kc);

      virtual void writeBuffer() = 0;
      virtual void init() = 0;
      virtual void reset() = 0;
      virtual std::auto_ptr<Observation> templateObservation() const = 0;

      virtual Observation::Type type() const = 0;

      static std::auto_ptr<ObservationWriter>
      createWriter(const std::string& arg, const Observation::Type t);

    };

  /** @brief Writes the content of @p kc using the provided writer @p w. */
  void writeObservations(ObservationWriter &w, const KernelCollection &kc);  
  /** @brief Writes the content of @p kc to file @p s, using file format @p t. */
  void writeObservations(const std::string &s, const KernelCollection &kc,
                         const Observation::Type &t = Observation::NUKLEI);
  /**
   * @brief Writes @p k to file @p s, using file format @p t.
   */
  void writeSingleObservation(const std::string &s, const kernel::base &k,
                              const Observation::Type &t = Observation::NUKLEI);  

}

#endif

