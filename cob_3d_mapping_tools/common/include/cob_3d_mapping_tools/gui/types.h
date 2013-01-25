/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_tools
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef COB_3D_MAPPING_TOOLS_GUI_TYPES_H_
#define COB_3D_MAPPING_TOOLS_GUI_TYPES_H_

#include <highgui.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


namespace Gui
{
  typedef cv::Mat_<cv::Vec3b> cvImage;
  typedef boost::shared_ptr<cvImage> cvImagePtr;
  typedef boost::shared_ptr<wxBitmap> wxBitmapPtr;

  namespace ViewTypes
  {
    struct View2D {};
    struct View3D {};
    struct ViewText {};

    struct Color : public View2D { static const std::string STR; };
    struct Depth_Z : public View2D { static const std::string STR; };
    template<size_t Channel>
    struct Normal : public View2D { static const std::string STR; };

    // not implemented, totally different approach necessary! use OnIdle in MainApp -> wxApp
    struct Depth_3D : public View3D { static const std::string STR; };
    struct Histogram : public View2D { static const std::string STR; }; // not implemented
    struct Curvature : public View2D { static const std::string STR; }; // not implemented
    struct SomethingWithText : public ViewText { static const std::string STR; }; // not implemented

    const std::string Color::STR = "Color View";
    const std::string Depth_Z::STR = "Depth View";
    template<> const std::string Normal<0>::STR = "Normal x View";
    template<> const std::string Normal<1>::STR = "Normal y View";
    template<> const std::string Normal<2>::STR = "Normal z View";
    const std::string Depth_3D::STR = "Depth3D View";
    const std::string Histogram::STR = "Histogram View";
    const std::string Curvature::STR = "Curvature View";
    const std::string SomethingWithText::STR = "Text View";
  }

  namespace ResourceTypes
  {
    struct BaseCloud {};

    template<typename PT>
    struct PointCloud : public BaseCloud
    {
      static const std::string STR;
      typedef pcl::PointCloud<PT> DataTypeRaw;
      typedef typename pcl::PointCloud<PT>::Ptr DataTypePtr;
      typedef PT PointType; // special typedef for all base clouds
    };

    template<typename PT>
    struct OrganizedPointCloud  : public BaseCloud
    {
      static const std::string STR;
      typedef pcl::PointCloud<PT> DataTypeRaw;
      typedef typename pcl::PointCloud<PT>::Ptr DataTypePtr;
      typedef PT PointType; // special typedef for all base clouds
    };

    struct Image
    {
      typedef cvImage DataTypeRaw;
      typedef cvImagePtr DataTypePtr;
      static const std::string STR;
    };

    template<typename PT> const std::string PointCloud<PT>::STR = "Cloud";
    template<typename PT> const std::string OrganizedPointCloud<PT>::STR = "OrgaCloud";
    const std::string Image::STR = "Image";
  }
}

#endif
