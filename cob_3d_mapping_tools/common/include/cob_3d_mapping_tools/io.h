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
 * Date of creation: 11/2011
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

#ifndef COB_3D_MAPPING_TOOLS_IO_H_
#define COB_3D_MAPPING_TOOLS_IO_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <float.h>

namespace cob_3d_mapping_tools
{
  class PPMReader
  {
  public:
    PPMReader()
    {
    }

    int mapRGB (const std::string &file_name, pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

  };
  
  class PPMWriter
  {
  public:
    PPMWriter()
      : fixed_max_(false), fixed_min_(false), max_z_(FLT_MIN), min_z_(FLT_MAX)
    {
    }

    int writeRGB (const std::string &file_name, const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

    int writeDepth (const std::string &file_name, const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

    void setMaxZ (const float &max);
    void setMinZ (const float &min);

    bool fixed_max_;
    bool fixed_min_;
    float max_z_;
    float min_z_;
  };

  uint32_t getGradientColor(double position, uint8_t rgb[]);
}

#endif // #ifndef COB_3D_MAPPING_TOOLS_IO_H_
