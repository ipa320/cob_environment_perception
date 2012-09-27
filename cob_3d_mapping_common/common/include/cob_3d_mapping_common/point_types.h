/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 08/2010
*
* \brief
* Definition of PCL point types
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef IPA_POINT_TYPES_H_
#define IPA_POINT_TYPES_H_

#include <pcl/ros/register_point_struct.h>
#include <pcl/point_types.h>
//#include <ros/ros.h>

using namespace pcl;

struct PointXYZCI;

struct PointXYZA;

struct PointXYZRGBF;

struct PointLabel;

#include "cob_3d_mapping_common/impl/point_types.hpp"

//Point obtained from Swissranger devices, confidence only used for SR2
POINT_CLOUD_REGISTER_POINT_STRUCT(
	PointXYZCI,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, confidence, confidence)
  (float, intensity, intensity));

//Point obtained from Camboard
POINT_CLOUD_REGISTER_POINT_STRUCT(
	PointXYZA,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, amplitude, amplitude));

POINT_CLOUD_REGISTER_POINT_STRUCT(
	PointXYZRGBF,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, rgb, rgb)
  (uint8_t, isF, isF));

POINT_CLOUD_REGISTER_POINT_STRUCT(
	PointLabel,
  (int, label, label));

#endif /* IPA_POINT_TYPES_H_ */
