// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_PCL_BRIDGE_H
#define NUKLEI_PCL_BRIDGE_H

#ifdef NUKLEI_USE_PCL

#include <nuklei/KernelCollection.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/math/special_functions/fpclassify.hpp>

// Forward declaration of PCL types
//namespace pcl {
//  template <typename PointT>
//  class PointCloud;
//  
//  struct PointXYZ;
//  //struct PointXYZI;
//  //struct PointXYZRGBA;
//  struct PointXYZRGB;
//  struct PointNormal;
//  struct PointXYZRGBNormal;
//  //struct PointXYZINormal;
//}

namespace nuklei {
  
  inline bool valid(const pcl::PointXYZ& p)
  {
    float t = p.x + p.y + p.z;
    return !(boost::math::isnan)(t);
  }

  inline bool valid(const pcl::PointNormal& p)
  {
    float t = p.x + p.y + p.z + p.normal_x + p.normal_y + p.normal_z;
    return !(boost::math::isnan)(t);
  }

  inline bool valid(const pcl::PointXYZRGB& p)
  {
    float t = p.x + p.y + p.z + p.rgb;
    return !(boost::math::isnan)(t);
  }

  inline bool valid(const pcl::PointXYZRGBNormal& p)
  {
    float t = p.x + p.y + p.z + p.normal_x + p.normal_y + p.normal_z + p.rgb;
    return !(boost::math::isnan)(t);
  }

  
  kernel::r3 nukleiKernelFromPclPoint(const pcl::PointXYZ& p);

  kernel::r3 nukleiKernelFromPclPoint(const pcl::PointXYZRGB& p);
  
  /**
   *
   *
   * Warning: converts to axial orientation. 
   */
  kernel::r3xs2p nukleiKernelFromPclPoint(const pcl::PointNormal& p);
  
  /**
   *
   *
   * Warning: converts to axial orientation. 
   */
  kernel::r3xs2p nukleiKernelFromPclPoint(const pcl::PointXYZRGBNormal& p);

  

  void pclPointFromNukleiKernel(pcl::PointXYZ& p, const kernel::r3& k);

  void pclPointFromNukleiKernel(pcl::PointXYZ& p, const kernel::base& k);

  
  void pclPointFromNukleiKernel(pcl::PointXYZRGB& p, const kernel::r3& k);
  
  void pclPointFromNukleiKernel(pcl::PointXYZRGB& p, const kernel::base& k);

  
  /**
   *
   *
   * Warning: converts to axial orientation. 
   */
  void pclPointFromNukleiKernel(pcl::PointNormal& p, const kernel::r3xs2p& k);
  
  void pclPointFromNukleiKernel(pcl::PointNormal& p, const kernel::base& k);

  
  /**
   *
   *
   * Warning: converts to axial orientation. 
   */
  void pclPointFromNukleiKernel(pcl::PointXYZRGBNormal& p, const kernel::r3xs2p& k);
  
  void pclPointFromNukleiKernel(pcl::PointXYZRGBNormal& p, const kernel::base& k);

  
  template<typename PointT>
  KernelCollection nukleiFromPcl(const pcl::PointCloud<PointT>& cloud,
                                 const bool removeInvalidPoints = false)
  {
    KernelCollection kc;
    for (typename pcl::PointCloud<PointT>::const_iterator i = cloud.begin();
         i != cloud.end(); ++i)
    {
      if (removeInvalidPoints && !valid(*i)) continue;
      kc.add(nukleiKernelFromPclPoint(*i));
    }
    return kc;
  }

  template<typename PointT>
  pcl::PointCloud<PointT> pclFromNuklei(const KernelCollection& kc)
  {
    pcl::PointCloud<PointT> cloud;
    
    cloud.width    = kc.size();
    cloud.height   = 1;
    cloud.is_dense = true; // all points coming from Nuklei should be valid.
        
    for (KernelCollection::const_iterator i = kc.begin();
         i != kc.end(); ++i)
    {
      PointT point;
      pclPointFromNukleiKernel(point, *i);
      cloud.push_back(point);
    }
    return cloud;
  }

}

#endif


#endif
