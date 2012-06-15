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
 * ROS package name: cob_3d_mapping_semantics
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
 * ToDo:
 * comments in doxygen style
 *
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

class TableExtraction
{

public:

  /**
   * @brief Constructor
   */
  TableExtraction ()
  :norm_x_min_(-0.1),
   norm_x_max_(0.1),
   norm_y_min_(-0.1),
   norm_y_max_(0.1),
   norm_z_min_ (-0.99),
   norm_z_max_ (0.99),
   height_min_ (0.4),
   height_max_ (1),
   area_min_ (1),
   area_max_ (3)
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

  void
  setNormalBounds (double tilt_angle)
  {
    static const double PI = 3.1415926;
    double ang_rad = tilt_angle * (PI / 180.0);
    double norm = cos ((PI/2)-ang_rad);
    setNormXMin (-norm);
    setNormXMax (norm);
    setNormYMin (-norm);
    setNormYMax (norm);
    setNormZMin (-cos (ang_rad));
    setNormZMax (cos (ang_rad));
    /*std::cout << "\n\t*tilt_angle = " << tilt_angle << std::endl;
    std::cout << "\n\t*norm_x_min = " << norm_x_min_ << std::endl;
    std::cout << "\n\t*norm_x_max = " << norm_x_max_ << std::endl;
    std::cout << "\n\t*norm_y_min = " << norm_y_min_ << std::endl;
    std::cout << "\n\t*norm_y_max = " << norm_y_max_ << std::endl;
    std::cout << "\n\t*norm_z_min = " << norm_z_min_ << std::endl;
    std::cout << "\n\t*norm_z_max = " << norm_z_max_ << std::endl;*/
  }

  /**
   * @brief Set Minimum threshold for x component of normal vector
   *
   * @param norm_x_min minimum threshold value
   *
   */
  void
  setNormXMin (double norm_x_min)
  {
    norm_x_min_ = norm_x_min;
  }

  /**
   * @brief Set Maximum threshold for x component of normal vector
   *
   * @param norm_x_max maximum threshold value
   *
   */
  void
  setNormXMax (double norm_x_max)
  {
    norm_x_max_ = norm_x_max;
  }

  /**
   * @brief Set Minimum threshold for y component of normal vector
   *
   * @param norm_y_min minimum threshold value
   *
   */
  void
  setNormYMin (double norm_y_min)
  {
    norm_y_min_ = norm_y_min;
  }

  /**
   * @brief Set Maximum threshold for y component of normal vector
   *
   * @param norm_y_max maximum threshold value
   *
   */
  void
  setNormYMax (double norm_y_max)
  {

    norm_y_max_ = norm_y_max;
  }

  /**
   * @brief Set Minimum threshold for z component of normal vector
   *
   * @param norm_z_min minimum threshold value
   *
   */
  void
  setNormZMin (double norm_z_min)
  {
    norm_z_min_ = norm_z_min;
  }

  /**
   * @brief Set Maximum threshold for z component of normal vector
   *
   * @param norm_z_max maximum threshold value
   *
   */
  void
  setNormZMax (double norm_z_max)
  {
    norm_z_max_ = norm_z_max;
  }
  /**
   * @brief Set Minimum threshold for height
   *
   * @param height_min minimum threshold value
   *
   */
  void
  setHeightMin (double height_min)
  {
    height_min_ = height_min;
  }

  /**
   * @brief Set Maximum threshold for height
   *
   * @param height_man maximum threshold value
   *
   */
  void
  setHeightMax (double height_max)
  {
    height_max_ = height_max;
  }
  /**
   * @brief Set Minimum threshold for area of a polygon
   *
   * @param area_min minimum threshold value
   *
   */
  void
  setAreaMin (double area_min)
  {
    area_min_ = area_min;
  }

  /**
   * @brief Set Maximum threshold for area of a polygon
   *
   * @param area_max maximum threshold value
   *
   */
  void
  setAreaMax (double area_max)
  {
    area_max_ = area_max;
  }

  /**
   * @brief sets the input polygon
   *
   * @param poly_ptr pointer to the polygon
   *
   */
  void setInputPolygon(PolygonPtr poly_ptr)
  {
    poly_ptr_ = poly_ptr;
  }

  /**
   * @brief check if the polygon is a table object candidate or not
   *
   * @return true or false
   */
  bool
  isTable();

  /**
   * @brief check if the plane of the polygon is horizontal or not
   *
   * @return true or false
   */

  bool
  isHorizontal ();

  /**
   * @brief check if the plane is high enough or not
   *
   * @return nothing
   */

  bool
  isHeightOk ();

  /**
   * @brief check if the area of the plane polygon is sufficient or not
   *
   * @return nothing
   */
  bool
  isSizeOk ();

protected:

  double norm_x_min_, norm_x_max_;
  double norm_y_min_, norm_y_max_;
  double norm_z_min_, norm_z_max_;

  double height_min_, height_max_;

  double area_min_, area_max_;

  PolygonPtr poly_ptr_;

};

#endif /* TABLE_EXTRACTION_H_ */
