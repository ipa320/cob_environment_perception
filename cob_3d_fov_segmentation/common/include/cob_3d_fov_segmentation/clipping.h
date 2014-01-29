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

#ifndef COB_3D_CLIPPING_H
#define COB_3D_CLIPPING_H

  /* 2D/3D line clipping algorithm
   * Source: http://en.wikipedia.org/wiki/Cohen-Sutherland (did not find a paper about it)
   */
namespace cob_3d_mapping
{
  class CohenSutherlandClipping
  {
  public:
    CohenSutherlandClipping()
      : xmin(-1.0f)
      , xmax( 1.0f)
      , ymin(-1.0f)
      , ymax( 1.0f)
      , zmin(-1.0f)
      , zmax( 1.0f)
    { }

    void setBorders(float x_min, float x_max,
                    float y_min, float y_max,
                    float z_min, float z_max)
    {
      xmin = x_min; xmax = x_max;
      ymin = y_min; ymax = y_max;
      zmin = z_min; zmax = z_max;
    }

    int computeCode(float x, float y, float z=0.0f)
    {
      int code = 0;
      if( x < xmin )     code |= LEFT;
      else if( x > xmax) code |= RIGHT;
      if( y < ymin )     code |= TOP;
      else if( y > ymax) code |= BOTTOM;
      if( z < zmin )     code |= NEAR;
      else if( z > zmax) code |= FAR;

      return code;
    }

    bool clip(const Eigen::Vector3f& p0,
              const Eigen::Vector3f& p1,
              Eigen::Vector3f& q0,
              Eigen::Vector3f& q1);


  private:
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;

    static const int FAR;    // 10 00 00
    static const int NEAR;   // 01 00 00
    static const int TOP;    // 00 10 00
    static const int BOTTOM; // 00 01 00
    static const int RIGHT;  // 00 00 10
    static const int LEFT;   // 00 00 01
  };

  const int CohenSutherlandClipping::FAR    = 32;
  const int CohenSutherlandClipping::NEAR   = 16;
  const int CohenSutherlandClipping::TOP    =  8;
  const int CohenSutherlandClipping::BOTTOM =  4;
  const int CohenSutherlandClipping::RIGHT  =  2;
  const int CohenSutherlandClipping::LEFT   =  1;
}

#include "cob_3d_fov_segmentation/impl/clipping.hpp"

#endif
