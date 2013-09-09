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
 *  ROS package name: cob_3d_fov_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2013
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

#ifndef COB_3D_PROJECTION_H
#define COB_3D_PROJECTION_H

#include <cob_3d_mapping_common/polygon.h>

namespace cob_3d_mapping
{
  template<typename Vec3T>
  inline Eigen::Vector4f makeVector4f(const Vec3T& v, float last=1.0f)
  {
    return Eigen::Vector4f(v[0], v[1], v[2], last);
  }

  template<typename SensorT>
  class PerspectiveProjection
  {
  private:
    typedef Eigen::Vector3f Vec3;
    typedef Eigen::Vector4f Vec4;
    typedef Eigen::Matrix4f Mat4;
    typedef typename SensorT::MatMap MatMap;

  public:
    /* specialized for cob_3d_mapping::Polygon
     * camera_transform: transformation from world to camera
     * w,h: size of projected image
     * polygons: input polygons
     * projection: image vector with polygon ids
     */
    static void compute(
      const Mat4& tf_camera, int w, int h,
      const std::vector<cob_3d_mapping::Polygon::Ptr>& polygons,
      std::vector<std::vector<int> >& projection);
  };
}

#include "cob_3d_fov_segmentation/impl/projection.hpp"

#endif
