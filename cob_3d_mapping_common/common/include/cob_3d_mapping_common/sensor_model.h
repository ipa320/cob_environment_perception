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
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

#include "cob_3d_mapping_common/polygon.h"
#include <list>
#include <algorithm>

namespace cob_3d_mapping
{
  /**
   * \brief Sensor model for PrimeSense cameras.
   */
  class PrimeSense
  {
  public:
    typedef Eigen::Map<const Eigen::Matrix<float,4,4,Eigen::RowMajor> > MatMap;

    /**
     * \brief Empty constructor.
     */
    PrimeSense() {}
    /**
     * \brief Empty destructor.
     */
    ~PrimeSense() {}

    /*
     * \brief Check if two points are neighbors.
     *
     * \param[in] query The depth of the query point.
     * \param[in] neighbor The depth of the potential neighbor.
     * \param[in] tolerance The distance tolerance factor.
     *
     * \return True, if the points are neighbors.
     */
    inline static bool areNeighbors(float query,
                             float neighbor,
                             float tolerance = 2.0f)
    {
      float dist_th = tolerance * 0.003f * query * query;
      //if (query < 1.2) dist_th += 0.01f;
      return (fabs(query - neighbor) < dist_th);
    }

    /*
     * \brief Check if two points are neighbors.
     *
     * \param[in] query The query point.
     * \param[in] neighbor The potential neighbor.
     * \param[in] tolerance The distance tolerance factor.
     *
     * \return True, if the points are neighbors.
     */
    inline static bool areNeighbors(const Eigen::Vector3f& query,
                             const Eigen::Vector3f&  neighbor,
                             float tolerance = 4.0f)
    {
      // += max( 0, 4-10*min(|x|,|y|) )
      tolerance += std::max( 0.0f, 1.0f * (4.0f - std::min(std::abs(query(0)),std::abs(query(1))) * 10.0f) );
      float dist_th =  tolerance * 0.003f * query(2) * query(2);
      return (std::abs(query(2) - neighbor(2)) < dist_th);
    }

    inline static Eigen::Matrix4f tf_unit_cube() { return MatMap(mat); }

    // frustum parameters:
    static const float fov_h; //!< fov horizontal (IR) , from wiki and ros.org
    static const float fov_v; //!< fov vertical (IR)
    static const float f; //!< far plane
    static const float n; //!< near plane
    static const float l; //!< left
    static const float r; //!< right
    static const float t; //!< top
    static const float b; //!< bottom
    static const float w;
    static const float h;
    static const float aspect;
    static const float mat[]; // transformation from view frustum to unit cube
  };

  const float PrimeSense::fov_h = 49.0f / 180.0f * M_PI;
  const float PrimeSense::fov_v = 43.0f / 180.0f * M_PI;
  const float PrimeSense::f = 5.0f;
  const float PrimeSense::n = 0.5f;
  const float PrimeSense::r = PrimeSense::n * tan(PrimeSense::fov_h * 0.5f);
  const float PrimeSense::l = - PrimeSense::r;
  const float PrimeSense::t = PrimeSense::n * tan(PrimeSense::fov_v * 0.5f);
  const float PrimeSense::b = - PrimeSense::t;
  const float PrimeSense::w = 640.0f;
  const float PrimeSense::h = 480.0f;
  const float PrimeSense::aspect = PrimeSense::w / PrimeSense::h;

  const float PrimeSense::mat[] =
  {
    1.0f / ( PrimeSense::aspect * tan(0.5f * PrimeSense::fov_h) ),
    0,
    0,
    0,

    0,
    1.0f / ( tan(0.5f * PrimeSense::fov_h) ),
    0,
    0,

    0,
    0,
    (PrimeSense::n + PrimeSense::f) / (PrimeSense::n - PrimeSense::f),
    -2.0f * PrimeSense::f * PrimeSense::n / (PrimeSense::n - PrimeSense::f),

    0,
    0,
    1,
    0
  };

}

#endif
