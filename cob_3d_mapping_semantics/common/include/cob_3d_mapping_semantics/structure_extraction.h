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

#ifndef STRUCTURE_EXTRACTION_H_
#define STRUCTURE_EXTRACTION_H_

//##################
//#### includes ####

//standard includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

//PCL includes
#include "pcl/point_types.h"
#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"

//other includes
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <cob_3d_mapping_common/polygon.h>

using namespace cob_3d_mapping;

class StructureExtraction
{

public:

  /**
   * @brief Constructor
   */
  StructureExtraction ()
    : floor_height_(0.2),
      ceiling_height_(2.2)
  {
    /// void
  }

  /**
   * @brief Destructor
   */
  ~StructureExtraction ()
  {

    /// void
  }

  /**
   * @brief Set the height of the floor
   *
   * @param height the floor height
   *
   */
  void
  setFloorHeight (double height)
  {
    floor_height_ = height;
  }

  /**
   * @brief Set the height of the ceiling
   *
   * @param height the ceiling height
   *
   */
  void
  setCeilingHeight (double height)
  {
    ceiling_height_ = height;
  }


  /**
   * @brief sets the input polygon
   *
   * @param poly_ptr pointer to the polygon
   *
   */
  void setInputPolygon(Polygon::Ptr poly_ptr)
  {
    poly_ptr_ = poly_ptr;
  }

  /**
   * @brief check if the polygon is a wall
   *
   * @return true or false
   */
  bool
  isWall();

  /**
   * @brief check if the polygon is a floor
   *
   * @return true or false
   */

  bool
  isFloor ();

  /**
   * @brief check if the polygon is a ceiling
   *
   * @return nothing
   */

  bool
  isCeiling ();

  /**
   * @brief classify the Polygon
   *
   * @return nothing
   */
  bool
  classify (unsigned int& label);

protected:

  double floor_height_, ceiling_height_;

  Polygon::Ptr poly_ptr_;

};

#endif /* STRUCTURE_EXTRACTION_H_ */
