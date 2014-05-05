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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_semantics
 *
 * \author
 *  Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2011
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

#include "cob_3d_mapping_semantics/table_extraction.h"

using namespace cob_3d_mapping;

bool
TableExtraction::isTable ()
{
  if (!poly_ptr_)
  {
    std::cerr << "Input polygon not set, aborting..." << std::endl;
    return false;
  }
  //Check if the plane spanned by the polygon is horizontal or not
  if (isHorizontal () && isHeightOk () && isSizeOk ())
    return true;
  else
    return false;
}

bool
TableExtraction::isHorizontal ()
{
  //check components of normal_ vector with threshold values
  if ((poly_ptr_->normal_ (2) >= norm_z_max_ || poly_ptr_->normal_ (2) <= norm_z_min_)
      && (poly_ptr_->normal_ (0) < norm_x_max_ && poly_ptr_->normal_ (0) > norm_x_min_)
      && (poly_ptr_->normal_ (1) < norm_y_max_ && poly_ptr_->normal_ (1) > norm_y_min_))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool
TableExtraction::isHeightOk ()
{
  if (poly_ptr_->pose_.translation () (2) > height_min_ && poly_ptr_->pose_.translation () (2) < height_max_)
    return true;
  else
    return false;
}

bool
TableExtraction::isSizeOk ()
{
  double area = poly_ptr_->computeArea3d ();
  if (area >= area_min_ && area <= area_max_)
    return true;
  return false;
}


