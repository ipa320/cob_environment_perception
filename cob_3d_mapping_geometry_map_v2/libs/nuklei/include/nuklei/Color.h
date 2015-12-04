// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_COLOR_H
#define NUKLEI_COLOR_H

#include <memory>
#include <boost/utility.hpp>
#include <nuklei/LinearAlgebra.h>
#include <nuklei/Common.h>
#include <nuklei/GenericKernel.h>

namespace nuklei {
  
  class Color : boost::noncopyable
  {
  public:
    
    typedef std::auto_ptr< Color > ptr;
    
    typedef enum { RGB = 0, HSV, HSVCONE, UNKNOWN } Type;
    static const Type defaultType = RGB;
    static const std::string TypeNames[];
    
    virtual ~Color() {}
    
    virtual void assertConsistency() const = 0;
        
    virtual std::auto_ptr<Color> clone() const = 0;
    virtual std::auto_ptr<Color> create() const = 0;
    
    virtual appear_t distanceTo(const Color& c) const = 0;
    virtual appear_t getMaxDist() const = 0;
    
    virtual void makeRandom() = 0;
    
    virtual GVector getVector() const = 0;
    virtual void setVector(const GVector &v) = 0;
    
  private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
    }
  };
  
  inline Color* new_clone(const Color& d)
  {
    return d.clone().release();
  }
  
  class RGBColor : public Color
  {
  public:
    typedef std::auto_ptr< RGBColor > ptr;
    
    RGBColor() : c_(0, 0, 0) {assertConsistency();}
    RGBColor(appear_t r, appear_t g, appear_t b) : c_(r, g, b) {assertConsistency();}
    RGBColor(const Vector3 &rgb) : c_(rgb) {assertConsistency();}
    //fixme: this is not good, change this to a method.
    // I don't remember what that comment is about.. :-/
    explicit RGBColor(const Color& c);
    
    std::auto_ptr<Color> clone() const { return std::auto_ptr<Color>(new RGBColor(c_)); }
    std::auto_ptr<Color> create() const { return std::auto_ptr<Color>(new RGBColor); }
    
    void assertConsistency() const
    {
      NUKLEI_RANGE_CHECK(R(), 0, 1);
      NUKLEI_RANGE_CHECK(G(), 0, 1);
      NUKLEI_RANGE_CHECK(B(), 0, 1);
    }
    
    appear_t& R() { return c_.X(); }
    appear_t R() const { return c_.X(); }
    appear_t& G() { return c_.Y(); }
    appear_t G() const { return c_.Y(); }
    appear_t& B() { return c_.Z(); }
    appear_t B() const { return c_.Z(); }
    
    Vector3 getRGB() const { return c_; }
    void setRGB(const Vector3& c) { c_ = c; }
    
    appear_t& at(unsigned i) { NUKLEI_ASSERT(0 <= i && i < 3); return c_[i]; }
    appear_t at(unsigned i) const { NUKLEI_ASSERT(0 <= i && i < 3); return c_[i]; }
    
    appear_t distanceTo(const Color& c) const
    {
      return distanceTo(dynamic_cast<const RGBColor&>(c));
    }
    appear_t distanceTo(const RGBColor& rgb) const
    {
      return (c_-rgb.c_).Length();
    }
    appear_t getMaxDist() const { return MAX_DIST; }
    
    void makeRandom();
    
    GVector getVector() const
    {
      return GVector(3, getRGB());
    }
    void setVector(const GVector &v)
    {
      c_ = la::vector3Copy(v);
    }
    
    static const appear_t MAX_DIST;
    
  private:
    Vector3 c_;
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
      ar & boost::serialization::make_nvp
      ( "base",  
       boost::serialization::base_object<Color>( *this ) );
      ar & BOOST_SERIALIZATION_NVP(c_);
    }
  };
  
  class HSVColor : public Color
  {
  public:
    typedef std::auto_ptr< HSVColor > ptr;
    
    HSVColor() : c_(0, 0, 0) {assertConsistency();}
    HSVColor(appear_t h, appear_t s, appear_t v) : c_(h, s, v) {assertConsistency();}
    HSVColor(const Vector3 &hsv) : c_(hsv) {assertConsistency();}
    explicit HSVColor(const Color& c);
    
    std::auto_ptr<Color> clone() const { return std::auto_ptr<Color>(new HSVColor(c_)); }
    std::auto_ptr<Color> create() const { return std::auto_ptr<Color>(new HSVColor); }
    
    void assertConsistency() const
    {
      NUKLEI_RANGE_CHECK(H(), 0, 2*M_PI);
      NUKLEI_RANGE_CHECK(S(), 0, 1);
      NUKLEI_RANGE_CHECK(V(), 0, 1);
    }
    
    appear_t& H() { return c_.X(); }
    appear_t H() const { return c_.X(); }
    appear_t& S() { return c_.Y(); }
    appear_t S() const { return c_.Y(); }
    appear_t& V() { return c_.Z(); }
    appear_t V() const { return c_.Z(); }
    
    Vector3 getHSV() const { return c_; }
    void setHSV(const Vector3& c) { c_ = c; }
    
    appear_t& at(unsigned i) { NUKLEI_ASSERT(0 <= i && i < 3); return c_[i]; }
    appear_t at(unsigned i) const { NUKLEI_ASSERT(0 <= i && i < 3); return c_[i]; }
    
    appear_t distanceTo(const Color& c) const;
    appear_t distanceTo(const HSVColor& hsv) const;
    appear_t getMaxDist() const { return MAX_DIST; }
    
    void makeRandom();
    
    GVector getVector() const
    {
      return GVector(3, getHSV());
    }
    void setVector(const GVector &v)
    {
      c_ = la::vector3Copy(v);
    }
    
    static const appear_t MAX_DIST;
    
  private:
    Vector3 c_;
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
      ar & boost::serialization::make_nvp
      ( "base",  
       boost::serialization::base_object<Color>( *this ) );
      ar & BOOST_SERIALIZATION_NVP(c_);
    }
  };
  
  class HSVConeColor : public Color
  {
  public:
    typedef std::auto_ptr< HSVConeColor > ptr;
    
    HSVConeColor() : c_(0, 0, 0), valueWeight_(1) {assertConsistency();}
    HSVConeColor(appear_t sch, appear_t ssh, appear_t weightedValue, appear_t valueWeight = 1) :
    c_(sch, ssh, weightedValue), valueWeight_(valueWeight) {assertConsistency();}
    HSVConeColor(const Vector3 &c, appear_t valueWeight = 1) : c_(c), valueWeight_(valueWeight) {assertConsistency();}
    explicit HSVConeColor(const Color& c);
    
    std::auto_ptr<Color> clone() const { return std::auto_ptr<Color>(new HSVConeColor(c_)); }
    std::auto_ptr<Color> create() const { return std::auto_ptr<Color>(new HSVConeColor); }
    
    void assertConsistency() const
    {
      NUKLEI_RANGE_CHECK(SCosH(), -1, 1);
      NUKLEI_RANGE_CHECK(SSinH(), -1, 1);
      NUKLEI_RANGE_CHECK(WV(), 0, 1);
    }
    
    appear_t& SCosH() { return c_.X(); }
    appear_t SCosH() const { return c_.X(); }
    appear_t& SSinH() { return c_.Y(); }
    appear_t SSinH() const { return c_.Y(); }
    appear_t& WV() { return c_.Z(); }
    appear_t WV() const { return c_.Z(); }
    appear_t& W() { return valueWeight_; }
    appear_t W() const { return valueWeight_; }
    
    Vector3 getHSVCone() const { return c_; }
    void setHSVCone(const Vector3& c) { c_ = c; }
    appear_t getValueWeight() const { return valueWeight_; }
    void setValueWeight(const appear_t& valueWeight) { valueWeight_ = valueWeight; }
    
    
    appear_t& at(unsigned i) { NUKLEI_ASSERT(0 <= i && i < 3); return c_[i]; }
    appear_t at(unsigned i) const { NUKLEI_ASSERT(0 <= i && i < 3); return c_[i]; }
    
    appear_t distanceTo(const Color& c) const;
    appear_t distanceTo(const HSVConeColor& hsvc) const;
    appear_t getMaxDist() const { return MAX_DIST; }
    
    void makeRandom();
    
    GVector getVector() const
    {
      GVector gv(4);
      for (int i = 0; i < 3; ++i) gv[i] = c_[i];
      gv[3] = valueWeight_;
      return gv;
    }
    void setVector(const GVector &v)
    {
      NUKLEI_ASSERT(v.GetSize() == 4);
      c_ = Vector3(v[0], v[1], v[2]);
      valueWeight_ = v[3];
    }
    
    static const appear_t MAX_DIST;
    
  private:
    Vector3 c_;
    appear_t valueWeight_;
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
      ar & boost::serialization::make_nvp
      ( "base",  
       boost::serialization::base_object<Color>( *this ) );
      ar & BOOST_SERIALIZATION_NVP(c_) & BOOST_SERIALIZATION_NVP(valueWeight_);
    }
  };
  
  
}

#endif
