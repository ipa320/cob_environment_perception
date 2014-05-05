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
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_semantics
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2012
 * ToDo:
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

#ifndef SUPPORTING_PLANE_EXTRACTION_H_
#define SUPPORTING_PLANE_EXTRACTION_H_

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

#include <cob_3d_mapping_common/polygon.h>

using namespace cob_3d_mapping;

/**
 * \brief Finds a supporting plane (e.g. a table top or a wall) in a polygon array.
 *
 * \details The supporting plane has an area the lies between a user-definied minimum and maximum value and
 * has to be in a certain distance range from the view point. The algorithm only works if a dominant plane exists in the array.
 * Works best with a polygon array defined in camera coordinates.
 */
class SupportingPlaneExtraction
{

public:

  /**
   * @brief Constructor
   */
  SupportingPlaneExtraction () :
      distance_min_ (0.4), distance_max_ (3), area_min_ (0.3), area_max_ (3)
  {
    /// void
  }

  /**
   * @brief Destructor
   */
  ~SupportingPlaneExtraction ()
  {

    /// void
  }

  /**
   * @brief Set the minimum threshold for the distance from the supporting plane to the view point.
   *
   * @param distance_min The minimum threshold.
   *
   */
  void
  setDistanceMin (double distance_min)
  {
    distance_min_ = distance_min;
  }

  /**
   * @brief Set the maximum threshold for the distance from the supporting plane to the view point.
   *
   * @param distance_man The maximum threshold.
   *
   */
  void
  setDistanceMax (double distance_max)
  {
    distance_max_ = distance_max;
  }

  /**
   * @brief Set the minimum threshold for the area of the supporting plane.
   *
   * @param area_min The minimum threshold.
   *
   */
  void
  setAreaMin (double area_min)
  {
    area_min_ = area_min;
  }

  /**
   * @brief Set the maximum threshold for the area of the supporting plane.
   *
   * @param area_max The maximum threshold.
   *
   */
  void
  setAreaMax (double area_max)
  {
    area_max_ = area_max;
  }

  /**
   * @brief Extract the supporting plane from a polygon array.
   *
   * \param[in] polys The polygon array to be evaluated.
   * \param[out] sp The supporting plane
   *
   * @return True if extraction was possible.
   */
  bool
  getSupportingPlane (std::vector<Polygon::Ptr>& polys, Polygon& sp);

protected:

  double distance_min_; ///< The minimum distance between supporting plane an view point.
  double distance_max_; ///< The minimum distance between supporting plane an view point.
  double area_min_; ///< The minimum area of the supporting plane.
  double area_max_; ///< The maximum area of the supporting plane.

};

#endif /* SUPPORTING_PLANE_EXTRACTION_H_ */
