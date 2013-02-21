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
* \date Date of creation: 02/2011
*
* \brief
* Computes field of view of camera sensors.
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


//##################
//#### includes ####

#include <cob_3d_fov_segmentation/field_of_view.h>


  void FieldOfView::computeFieldOfView()
  {
    double fovHorFrac = sensor_fov_hor_/2;
    double fovVerFrac = sensor_fov_ver_/2;

    p_1_cam_(0) = p_0_(0) + tan(fovHorFrac)*sensor_max_range_;
    p_1_cam_(1) = -tan(fovVerFrac)*sensor_max_range_;
    p_1_cam_(2) = sensor_max_range_;

    p_2_cam_(0) = -p_1_cam_(0);
    p_2_cam_(1) = p_1_cam_(1);
    p_2_cam_(2) = sensor_max_range_;

    p_3_cam_(0) = -p_1_cam_(0);
    p_3_cam_(1) = -p_1_cam_(1);
    p_3_cam_(2) = sensor_max_range_;

    p_4_cam_(0) = p_1_cam_(0);
    p_4_cam_(1) = -p_1_cam_(1);
    p_4_cam_(2) = sensor_max_range_;
  }
