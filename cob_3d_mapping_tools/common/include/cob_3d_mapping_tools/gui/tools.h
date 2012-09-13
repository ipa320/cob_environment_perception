/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2012
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef COB_3D_MAPPING_TOOLS_GUI_TOOLS_H_
#define COB_3D_MAPPING_TOOLS_GUI_TOOLS_H_

#include <highgui.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include "cob_3d_mapping_tools/gui/types.h"

namespace Gui
{
  namespace Tools
  {
    uint32_t getGradientColor(float, float, float, cv::Vec3b&);
    uint32_t getGradientColor(double, cv::Vec3b&);

    /* --- converter base class --- */
    template<typename PointT>
    class PointConverterBase
    {
      public:
      PointConverterBase(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
        : cloud_(cloud) { }
      virtual ~PointConverterBase() { }

      virtual cv::Vec3b operator() (int)=0;

      protected:
      typename pcl::PointCloud<PointT>::ConstPtr cloud_;
    };

    /* --- RGB converter --- */
    template<typename PointT>
    class PointConverterRGB : public PointConverterBase<PointT>
    {
      public:
      PointConverterRGB(const typename pcl::PointCloud<PointT>::ConstPtr& cloud) : PointConverterBase<PointT>(cloud) { }

      cv::Vec3b operator() (int idx)
      {
        cv::Vec3b rgb;
        rgb[0] = (*this->cloud_)[idx].b;
        rgb[1] = (*this->cloud_)[idx].g;
        rgb[2] = (*this->cloud_)[idx].r;
        return rgb;
      }
    };

    /* --- Depth converter --- */
    template<typename PointT>
    class PointConverterDepth : public PointConverterBase<PointT>
    {
      public:
      PointConverterDepth(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float z_min, float z_max)
        : PointConverterBase<PointT>(cloud), z_min_(z_min), z_max_(z_max)
      { }

      cv::Vec3b operator() (int idx)
      {
        cv::Vec3b bgr;
        getGradientColor((*this->cloud_)[idx].z, z_min_, z_max_, bgr);
        return bgr;
      }

      private:
      float z_min_;
      float z_max_;
    };

    template<typename PointT>
    void convertPointCloud2Cv(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, cvImagePtr& image,
                                boost::function<cv::Vec3b (int)>& converter);

    template<typename PointT>
    void getMinMaxZ(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float& z_min, float& z_max);
  }
}

#endif
