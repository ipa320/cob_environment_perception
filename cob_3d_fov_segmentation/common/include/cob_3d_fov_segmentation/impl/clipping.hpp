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

#ifndef COB_3D_CLIPPING_HPP
#define COB_3D_CLIPPING_HPP

namespace cob_3d_mapping
{
  bool CohenSutherLandClipping::clip(const Eigen::Vector3f& p0,
                                     const Eigen::Vector3f& p1,
                                     Eigen::Vector3f& q0,
                                     Eigen::Vector3f& q1)
  {
    int code0 = computeCode(p0(0),p0(1),p0(2));
    int code1 = computeCode(p1(0),p1(1),p1(2));
    bool accept = false;

    if( !(code0 | code1) ) // trivial accept
    {
      accept = true;
      q0 = p0;
      q1 = p1;
    }
    else if( !(code0 & code1) )
    {
      /* Line:  q = (p1 - p0) * d + p0
       * Plane: (q - c).dot(n)           c: point on plane, n: normal of plane
       *
       *           (p0 - c).dot(n)
       * => d =  ------------------
       *          (p1 - p0).dot(n)
       */
      float x, y, z, d;
      int outcode;
      Eigen::Vector3f *pnew;
      const Eigen::Vector3f *pold;
      if(code0)
      { // check which point lies outside
        outcode = code0;
        pnew = &q0;
        pold = &p1;
      }
      else
      {
        outcode = code1;
        pold = &p0;
        pnew = &q1;
      }

      if(outcode & FAR)
      {
        d = (zmax - p0(2)) / (p1(2) - p0(2));
        (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
        (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
        (*pnew)(2) = zmax;
      }
      else if(outcode & NEAR)
      {
        d = (zmin - p0(2)) / (p1(2) - p0(2));
        (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
        (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
        (*pnew)(2) = zmin;
      }
      else if(outcode & BOTTOM)
      {
        d = (ymax - p0(1)) / (p1(1) - p0(1));
        (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
        (*pnew)(1) = ymax;
        (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
      }
      else if(outcode & TOP)
      {
        d = (ymin - p0(1)) / (p1(1) - p0(1));
        (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
        (*pnew)(1) = ymin;
        (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
      }
      else if(outcode & RIGHT)
      {
        d = (xmax - p0(0)) / (p1(0) - p0(0));
        (*pnew)(0) = xmax;
        (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
        (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
      }
      else /*(outcode & LEFT)*/
      {
        d = (xmin - p0(0)) / (p1(0) - p0(0));
        (*pnew)(0) = xmin;
        (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
        (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
      }
      accept = clip(*pold,*pnew,q0,q1);
    }

    return accept;
  }
}
