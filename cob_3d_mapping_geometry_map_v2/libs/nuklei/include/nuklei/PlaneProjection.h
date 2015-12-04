// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_PLANE_PROJECTION_H
#define NUKLEI_PLANE_PROJECTION_H

#include <nuklei/LinearAlgebra.h>
#include <nuklei/Common.h>
#include <nuklei/member_clone_ptr.h>

#ifdef NUKLEI_USE_CIMG
// CImg is slow to compile. CImg-related methods will be put in a
// separate .cpp, which should include few headers (no KernelCollection.h, ...)
// so that it's not compiled too often.
namespace cimg_library
{
  template<typename T> class CImg;
}
#endif

namespace nuklei {

  class Color;

  struct PlaneProjection
  {
    typedef unsigned char pixel_t;
#ifdef NUKLEI_USE_CIMG
    typedef cimg_library::CImg<pixel_t> image_t;
#else
    typedef int image_t;
#endif
    
    PlaneProjection(const std::string& opencvStereoCalibFile, unsigned camNum = 0);
    PlaneProjection(const PlaneProjection& pp);
    PlaneProjection& operator=(const PlaneProjection& pp);
    ~PlaneProjection();
    
    Vector3 getCamPosition() const;
    Quaternion getCamOrientation() const;
    
    std::vector<Vector2> project(const std::vector<Vector3> worldPoints) const;
    Vector2 project(const Vector3& p) const;
    
    void readImage(const std::string& name);
    void writeImage(const std::string& name) const;
    
    const std::auto_ptr<Color> getColor() const;
    void setColor(const Color& c);
    
    weight_t getOpacity() const { return opacity_; }
    void setOpacity(const weight_t opacity) { opacity_ = opacity; }
    
    const image_t& getImage() const {
#ifdef NUKLEI_USE_CIMG
      NUKLEI_ASSERT(image_.get() != NULL); return *image_;
#else
      NUKLEI_THROW("This function requires CIMG.");
#endif
    }
    
  private:
    void readParameters(const std::string& opencvStereoCalibFile, unsigned camNum);
    
    std::vector<double> rotationMatrixData_, translationVectorData_,
      intrinsicMatrixData_, distortionCoeffsData_;
    
    
    pixel_t color_[3];
    float opacity_;
#ifdef NUKLEI_USE_CIMG
    std::auto_ptr<image_t> image_;
#endif
  };

  struct StereoPlaneProjection
  {
    typedef unsigned char pixel_t;
#ifdef NUKLEI_USE_CIMG
    typedef cimg_library::CImg<pixel_t> image_t;
#else
    typedef int image_t;
#endif
    
    StereoPlaneProjection(const std::string& opencvStereoCalibFile) :
      left_(opencvStereoCalibFile, 0),
      right_(opencvStereoCalibFile, 1)
    {}
    StereoPlaneProjection(const PlaneProjection &left, const PlaneProjection &right) :
      left_(left),
      right_(right)
    {}
    ~StereoPlaneProjection() {}
    
    void project(const std::vector<Vector3> worldPoints) const
    {
      left_.project(worldPoints);
      right_.project(worldPoints);
    }
    
    void readLeftImage(const std::string& name)
    {
      left_.readImage(name);
    }
    void readRightImage(const std::string& name)
    {
      right_.readImage(name);
    }
    void writeStereoImage(const std::string& name) const;
    
    const std::auto_ptr<Color> getColor() const;
    void setColor(const Color& c);
    
    weight_t getOpacity() const { return opacity_; }
    void setOpacity(const weight_t opacity)
    { opacity_ = opacity; left_.setOpacity(opacity_); right_.setOpacity(opacity_); }
    
  private:
    pixel_t color_[3];
    float opacity_;
    PlaneProjection left_, right_;
  };


}

#endif
