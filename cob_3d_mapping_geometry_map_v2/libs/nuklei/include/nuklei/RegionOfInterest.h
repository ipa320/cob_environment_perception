// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_REGIONOFINTEREST_H
#define NUKLEI_REGIONOFINTEREST_H


#include <nuklei/Definitions.h>
#include <nuklei/LinearAlgebra.h>
#include <nuklei/GenericKernel.h>
#include <boost/any.hpp>

namespace nuklei {
  
  class RegionOfInterest
  {
  public:
    RegionOfInterest(bool positive = true) : positive_(positive) {}
    
    virtual ~RegionOfInterest() {}
    
    bool contains(const Vector3 &v) const
    {
      return (positive_ == contains_(v)) || (next_ && next_->contains(v));
    }
    
    virtual void enqueue(boost::shared_ptr<RegionOfInterest> &roi)
    {
      if (next_)
        next_->enqueue(roi);
      else
        next_ = roi;
    }
    
    virtual void setSign(bool positive)
    {
      positive_ = positive;
    }
    
    void buildAABBTree()
    {
      buildAABBTree_();
    }

    void pushTriangles(std::list<boost::any>& triangles) const
    {
      if (next_)
        next_->pushTriangles(triangles);
      pushTriangles_(triangles);
    }
    
    virtual bool intersectsPlane(const Vector3& p,
                                 const Vector3& q,
                                 const Vector3& r) const
    { NUKLEI_THROW("Not implemented"); }
    virtual bool intersectsPlane(const Vector3& p, const Vector3& v) const
    { NUKLEI_THROW("Not implemented"); }
  
  protected:
    virtual bool contains_(const Vector3 &v) const = 0;
    virtual void buildAABBTree_()
    { NUKLEI_THROW("Not implemented"); }
    virtual void pushTriangles_(std::list<boost::any>& triangles) const
    { NUKLEI_THROW("Not implemented"); }
    bool positive_;
  private:
    boost::shared_ptr<RegionOfInterest> next_;
  };
  
  
  class SphereROI : public RegionOfInterest
  {
  public:
    SphereROI(const Vector3 &center, double radius) :
    center_(center), radius_(radius) {}
    
    SphereROI(const std::string &centerAndRadius);
    
    ~SphereROI() {}
  protected:
    bool contains_(const Vector3 &v) const;
  private:
    Vector3 center_;
    double radius_;
  };
  
  class BoxROI : public RegionOfInterest
  {
  public:
    BoxROI(const Vector3 &centerLoc, const Quaternion &centerOri,
           const Vector3 &edgeLengths) :
    centerLoc_(centerLoc), centerOri_(centerOri), edgeLengths_(edgeLengths) {}
    
    BoxROI(const std::string &s);
    
    void setCenterOriSize(const std::string &centerSize);
    void setCenterAxesSize(const std::string &centerSize);
    
    virtual bool intersectsPlane(const Vector3& p,
                                 const Vector3& q,
                                 const Vector3& r) const;
    virtual bool intersectsPlane(const Vector3& p, const Vector3& v) const;

    ~BoxROI() {}
  protected:
    bool contains_(const Vector3 &v) const;
    void buildAABBTree_();
    void pushTriangles_(std::list<boost::any>& triangles) const;
  private:
    Vector3 centerLoc_;
    Quaternion centerOri_;
    Vector3 edgeLengths_;
    boost::any tree_;
  };
  
}

#endif

