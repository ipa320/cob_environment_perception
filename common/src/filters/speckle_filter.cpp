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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2011

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
#ifndef SPECKLE_FILTER_HPP_
#define SPECKLE_FILTER_HPP_

//##################
//#### includes ####


#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"

// cob_env_model includes
#include "cob_env_model/filters/speckle_filter.h"
#include "cob_env_model/filters/impl/speckle_filter.hpp"
#include <cob_env_model/cpc_point.h>

#include <opencv/cv.h>
#include <cob_vision_utils/VisionUtils.h>

void
cob_env_model::SpeckleFilter<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &pc_out)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(pc_out,pc);

  cob_env_model::SpeckleFilter<pcl::PointXYZ>::applyFilter(pc);

  for(size_t i=0; i<pc.size(); i++) {
    memcpy (&pc_out->points[i].x, &pc.points[i].x, 3 * sizeof(float));
  }

}

using namespace pcl;
PCL_INSTANTIATE(SpeckleFilter, (CPCPoint));
