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

#ifndef COB_3D_MAPPING_TOOLS_GUI_TOOL_HELPERS_H_
#define COB_3D_MAPPING_TOOLS_GUI_TOOL_HELPERS_H_

#include <limits>

#include <highgui.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>

namespace cob_3d_mapping_tools
{
  namespace Gui
  {
    uint32_t getGradientColor(float, float, float, cv::Vec3b&);
    uint32_t getGradientColor(double, cv::Vec3b&);

    /* -------------------------
       templated helper classes
       ------------------------- */

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


    /* -------------------------
       templated helper functions
       ------------------------- */

    template<typename PointT>
    void convertPointCloud2Cv(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, cv::Mat& image,
                              boost::function<cv::Vec3b (int)>& converter)
    {
      image.create(cloud->height, cloud->width, CV_8UC3);
      cv::Mat_<cv::Vec3b>& ptr = (cv::Mat_<cv::Vec3b>&)image;
      int idx=0;
      for (int row = 0; row < image.rows; row++)
      {
        //ptr = image.ptr<unsigned char>(row);
        for (int col = 0; col < image.cols; col++, idx++)
        {
          ptr(row, col) = converter(idx);
        }
      }
    }

    template<typename PointT>
    void getMinMaxZ(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float& z_min, float& z_max)
    {
      z_min = std::numeric_limits<float>::max();
      z_max = std::numeric_limits<float>::min();
      for(typename pcl::PointCloud<PointT>::const_iterator it=cloud->begin(); it!=cloud->end(); ++it)
      {
        z_min = std::min(z_min,it->z);
        z_max = std::max(z_max,it->z);
      }
    }


    /* -------------------------
       free helper functions
       ------------------------- */

    uint32_t getGradientColor(float z_value, float z_min, float z_max, cv::Vec3b& bgr)
    {
      return getGradientColor( (z_value - z_min) / (z_max - z_min), bgr );
    }

    // color is proportional to position  <0;1>
    // position means position of color in color gradient
    uint32_t getGradientColor(double position, cv::Vec3b& bgr)
    {
      //if (position > 1) position = position - int(position);
      if (position > 1) position = 1;
      if (position < 0) position = 0;
      // if position > 1 then we have repetition of colors
      // it maybe useful
      int n_bars_max = 4;
      double m=n_bars_max * position;
      int n=int(m); // integer of m
      double f=m-n;  // fraction of m
      uint8_t t=int(f*255);

      switch (n)
      {
      case 0:
      { bgr[2] = 255; bgr[1] = t; bgr[0] = 0; break; }
      case 1:
      { bgr[2] = 255 - t; bgr[1] = 255; bgr[0] = 0; break; }
      case 2:
      { bgr[2] = 0; bgr[1] = 255; bgr[0] = t; break; }
      case 3:
      { bgr[2] = 0; bgr[1] = 255 - t; bgr[0] = 255; break; }
      case 4:
      { bgr[2] = t; bgr[1] = 0; bgr[0] = 255; break; }
      case 5:
      { bgr[2] = 255; bgr[1] = 0; bgr[0] = 255 - t; break; }
      };
      return (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
    }
  }
}

#endif
