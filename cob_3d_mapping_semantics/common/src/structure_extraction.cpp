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
 *  ROS package name: cob_3d_mapping_semantics
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 01/2013
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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
//internal include
#include "cob_3d_mapping_semantics/structure_extraction.h"
#include <ros/console.h>

using namespace cob_3d_mapping;

bool
StructureExtraction::isWall()
{
  //Check if the plane spanned by the polygon is horizontal or not
  if (fabs(poly_ptr_->normal_(2)) < 0.1 && poly_ptr_->computeArea3d() > 1)
    return true;
  else
    return false;
}

bool
StructureExtraction::isFloor ()
{
  if (fabs(poly_ptr_->normal_(0)) < 0.12 && fabs(poly_ptr_->normal_(1)) < 0.12 && fabs(poly_ptr_->normal_(2)) > 0.9 && poly_ptr_->pose_.translation()(2) <= floor_height_)
    return true;
  else
    return false;
}

bool
StructureExtraction::isCeiling ()
{
  if (fabs(poly_ptr_->normal_(0)) < 0.12 && fabs(poly_ptr_->normal_(1)) < 0.12 && fabs(poly_ptr_->normal_(2)) > 0.9 && poly_ptr_->pose_.translation()(2) >= ceiling_height_)
    return true;
  else
    return false;
}


bool
StructureExtraction::classify (unsigned int& label)
{
  if(!poly_ptr_)
  {
    ROS_ERROR("Input polygon not set, aborting...");
    return false;
  }
  if( isCeiling()) label = 1;
  else if( isFloor()) label = 2;
  else if( isWall() ) label = 3;
  else label = 0;
}




