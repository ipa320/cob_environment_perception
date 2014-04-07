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
 * ROS package name: cob_3d_fov_segmentation
 *
 * \author
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 02/2013
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

#ifndef FIELD_OF_VIEW_H__
#define FIELD_OF_VIEW_H__

//##################
//#### includes ####

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

namespace cob_3d_mapping
{

  /*
   * \brief Class to compute the field of view of a camera
   */
  class FieldOfView
  {
  public:
    /** Constructor */
    FieldOfView () :
        p_0_cam_ (0, 0, 0)
    {
      computeFieldOfView ();
    }

    /** Empty Destructor */
    ~FieldOfView ()
    {
      /// void
    }

    /**
     * \brief Set the horizontal FOV angle of the camera (degree).
     *
     * \param[in] val The angle in degree.
     */
    void
    setSensorFoV_hor (double val)
    {
      sensor_fov_hor_ = val * M_PI / 180;
    }

    /**
     * \brief Set the vertical FOV angle of the camera (degree).
     *
     * \param[in] val The angle in degree.
     */
    void
    setSensorFoV_ver (double val)
    {
      sensor_fov_ver_ = val * M_PI / 180;
    }

    /**
     * \brief Set the maximum range of the camera (meter).
     *
     * \param[in] val The maximum range in meter.
     */
    void
    setSensorMaxRange (double val)
    {
      sensor_max_range_ = val;
    }

    /**
     * \brief Return the FOV.
     *
     * \param[out] p_0 Origin point.
     * \param[out] p_1 First corner point of the frustum far plane.
     * \param[out] p_2 Second corner point of the frustum far plane.
     * \param[out] p_3 Third corner point of the frustum far plane.
     * \param[out] p_4 Forth corner point of the frustum far plane.
     *
     */
    void
    getFOV (Eigen::Vector3d& p_0, Eigen::Vector3d& p_1, Eigen::Vector3d& p_2, Eigen::Vector3d& p_3,
            Eigen::Vector3d& p_4)
    {
      p_0 = p_0_;
      p_1 = p_1_;
      p_2 = p_2_;
      p_3 = p_3_;
      p_4 = p_4_;
    }

    /**
     * @brief calculates coordinate system from parameters
     *
     * calculates coordinate system from parameters
     *
     * @return nothing
     */
    void
    computeFieldOfView ();

    /**
     * @brief uses global transformation (of the robot) to recalculate the field of view
     *
     * uses global transformation (of the robot) to recalculate the field of view (all vectors)
     *
     * @param stamp timestamp indicating used frame
     * @param target_frame targetframe
     *
     * @return nothing
     */
    void
    transformFOV (Eigen::Affine3d& t)
    {
      p_0_ = t * p_0_cam_;
      p_1_ = t * p_1_cam_;
      p_2_ = t * p_2_cam_;
      p_3_ = t * p_3_cam_;
      p_4_ = t * p_4_cam_;
    }

  protected:
    double sensor_fov_hor_; /// horizontal angle of sensor
    double sensor_fov_ver_; /// vertical angle of sensor
    double sensor_max_range_; /// maximum range of sensor

    Eigen::Vector3d p_0_; /// part of view frustum
    Eigen::Vector3d p_1_; /// part of view frustum
    Eigen::Vector3d p_2_; /// part of view frustum
    Eigen::Vector3d p_3_; /// part of view frustum
    Eigen::Vector3d p_4_; /// part of view frustum

    Eigen::Vector3d p_0_cam_; /// part of view frustum
    Eigen::Vector3d p_1_cam_; /// part of view frustum
    Eigen::Vector3d p_2_cam_; /// part of view frustum
    Eigen::Vector3d p_3_cam_; /// part of view frustum
    Eigen::Vector3d p_4_cam_; /// part of view frustum

  };

} //namespace

#endif

