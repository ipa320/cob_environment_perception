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

#ifndef SEMANTIC_EXTRACTION_H_
#define SEMANTIC_EXTRACTION_H_

//##################
//#### includes ####

//standard includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

//other includes
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

class SemanticExtraction
{

public:
  /**
   * @brief Contains polygon normal vector, d and polygon points
   */
  struct Polygon
  {
    std::vector<std::vector<Eigen::Vector3f> > poly_points;
    Eigen::Vector3f normal;
    float d;
  };

  /**
   * @brief A 3D point
   */
  struct Point
  {
    double x;
    double y;
    double z;
  };

  typedef boost::shared_ptr<Polygon> PolygonPtr;

  /**
   * @brief Constructor
   */
  SemanticExtraction ()
  {
    area_.resize (1);
  }

  /**
   * @brief Destructor
   */
  ~SemanticExtraction ()
  {

    /// void
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
  setHightMin (double height_min)
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
  setHightMax (double height_max)
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
   * @brief checks if the plane of the polygon is horizontal or not
   *
   * @param p_ptr shared pointer to Polygon struct
   *
   * @return true or false
   */

  bool
  isHorizontal (PolygonPtr p_ptr);

  /**
   * @brief checks if the plane is high enough or not
   *
   * @param p_ptr shared pointer to Polygon struct
   *
   * @return true or flase
   */

  bool
  isHeightOk (PolygonPtr p_ptr);

  /**
   * @brief checks if the area of the plane is sufficient or not
   *
   * @param p_ptr shared pointer to Polygon struct
   *
   * @return true or flase
   */

  bool
  isSizeOk (PolygonPtr p_ptr);

  /**
   * @brief Calculates area of a polygon
   *
   * @param p_ptr shared pointer to the polygon
   *
   * @return nothing
   */
  void
  calcPolyArea (PolygonPtr p_ptr);

  /**
   * @brief Calculates centroid of a polygon
   *
   * @param p_ptr shared pointer to the polygon
   *
   * @return nothing
   */
  void
  calcPolyCentroid (PolygonPtr p_ptr);

protected:

  double norm_x_min_, norm_x_max_;
  double norm_y_min_, norm_y_max_;
  double norm_z_min_, norm_z_max_;

  double height_min_, height_max_;

  double area_min_, area_max_;

  std::vector<double> area_;
  std::vector<Point> centroid_;

};

#endif /* SEMANTIC_EXTRACTION_H_ */
