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

#ifndef COB_3D_MAPPING_TOOLS_GUI_TOOL_H_
#define COB_3D_MAPPING_TOOLS_GUI_TOOL_H_

#include <boost/bind.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cob_3d_mapping_tools/impl/gui_tool_helpers.hpp"

namespace cob_3d_mapping_tools
{
  namespace Gui
  {
    template<typename PointT>
      class GuiTool
    {
    public:
      typedef typename pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      GuiTool()
        : z_min_(0.0)
        , z_max_(0.0)
        , enable_z_min_max_(false)
      { }

      void setPointCloud(const PointCloudConstPtr& cloud);
      void enableDepthMinMaxSlider(bool enable) { enable_z_min_max_ = enable; }
      void updateRGB();
      void updateDepth();
      void spin();
      void init();

    private:
      static void cb_depth_trackbar(int value, void* userdata);
      static void cb_mouse(int event, int x, int y, int flags, void* userdata);

    private:
      PointCloudConstPtr cloud_;
      cv::Mat cvColor_;
      cv::Mat cvDepth_;
      float z_min_;
      float z_max_;
      float z_diff_;
      bool enable_z_min_max_;
      int z_focus_;
      int z_range_;
    };
  }
}

#include "cob_3d_mapping_tools/impl/gui_tool.hpp"

#endif
