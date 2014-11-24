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

#ifndef TABLE_EXTRACTION_H_
#define TABLE_EXTRACTION_H_

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

/**
 * \brief Extracts table tops from a polygon array.
 */
class TableExtraction
{

public:

  /**
   * @brief Constructor
   */
  TableExtraction () :
      norm_x_min_ (-0.1), norm_x_max_ (0.1), norm_y_min_ (-0.1), norm_y_max_ (0.1), norm_z_min_ (-0.99), norm_z_max_ (
          0.99), height_min_ (0.4), height_max_ (1), area_min_ (1), area_max_ (3)
  {
    /// void
  }

  /**
   * @brief Destructor
   */
  ~TableExtraction ()
  {

    /// void
  }

  /**
   * \brief Set the allowed normal vector bounds for a given tilt angle.
   *
   * \param[in] tilt_angle The allowed tilt angle (in degrees).
   */
  void
  setNormalBounds (double tilt_angle)
  {
    static const double PI = 3.1415926;
    double ang_rad = tilt_angle * (PI / 180.0);
    double norm = cos ((PI / 2) - ang_rad);
    norm_x_min_ = -norm;
    norm_x_max_ = norm;
    norm_y_min_ = -norm;
    norm_y_max_ = norm;
    norm_z_min_ = -cos (ang_rad);
    norm_z_max_ = cos (ang_rad);
  }

  /**
   * @brief Set the minimum height of the table top.
   *
   * @param height_min The minimum height.
   *
   */
  void
  setHeightMin (double height_min)
  {
    height_min_ = height_min;
  }
  
  inline double getHeightMin() const {return height_min_;}

  /**
   * @brief Set the maximum height of the table top.
   *
   * @param height_man The maximum height.
   *
   */
  void
  setHeightMax (double height_max)
  {
    height_max_ = height_max;
  }
  
  inline double getHeightMax() const {return height_max_;}

  /**
   * @brief Set the minimum area of the table top.
   *
   * @param area_min The minimum area.
   *
   */
  void
  setAreaMin (double area_min)
  {
    area_min_ = area_min;
  }

  /**
   * @brief Set the maximum area of the table top.
   *
   * @param area_max The maximum area.
   *
   */
  void
  setAreaMax (double area_max)
  {
    area_max_ = area_max;
  }

  /**
   * @brief Set the input polygon.
   *
   * @param poly_ptr Pointer to the polygon.
   *
   */
  void
  setInputPolygon (Polygon::Ptr poly_ptr)
  {
    poly_ptr_ = poly_ptr;
  }

  /**
   * @brief Check if the polygon is a table top.
   *
   * @return True if it is a table top.
   */
  bool
  isTable ();

protected:

  /**
   * @brief Check if the plane of the polygon is horizontal.
   *
   * @return True if it is horizontal.
   */

  bool
  isHorizontal ();

  /**
   * @brief Check if the height of the plane polygon is within bounds.
   *
   * @return True if it is within bounds.
   */

  bool
  isHeightOk ();

  /**
   * @brief Check if the area of the plane polygon is within bounds.
   *
   * @return True if it is within bounds.
   */
  bool
  isSizeOk ();

  double norm_x_min_; ///< The minimum x component of the plane normal.
  double norm_x_max_; ///< The maximum x component of the plane normal.
  double norm_y_min_; ///< The minimum y component of the plane normal.
  double norm_y_max_; ///< The maximum y component of the plane normal.
  double norm_z_min_; ///< The minimum z component of the plane normal.
  double norm_z_max_; ///< The maximum z component of the plane normal.

  double height_min_; ///< The minimum height of the table.
  double height_max_; ///< The maximum height of the table.

  double area_min_; ///< The minimum area of the table.
  double area_max_; ///< The maximum area of the table.

  Polygon::Ptr poly_ptr_; ///< The polygon to be evaluated.

};

#endif /* TABLE_EXTRACTION_H_ */
