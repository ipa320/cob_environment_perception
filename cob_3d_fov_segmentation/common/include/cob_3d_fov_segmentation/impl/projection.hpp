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

#ifndef COB_3D_PROJECTION_HPP
#define COB_3D_PROJECTION_HPP

#include "cob_3d_fov_segmentation/scanline_polygon_fill.h"

namespace cob_3d_mapping
{
  template<typename SensorT>
  void PerspectiveProjection<SensorT>::compute(
    const Mat4& tf_camera, int w, int h,
    const std::vector<cob_3d_mapping::Polygon::Ptr>& polygons,
    std::vector<std::vector<int> >& projection)
  {
    // create transformation matrix from sensor position and camera parameters
    Mat4 proj = SensorT::tf_unit_cube() * tf_camera;

    // transform polygons to clipping space and reject outliers
    // ( space: x,y,z E [-1,1] )
    ScanlinePolygonFill spf(w,h);
    Vec4 projected;
    float xprev, yprev, xcurr, ycurr;
    for(int i=0; i<polygons.size(); ++i)
    {
      int id = polygons[i]->id_;
      std::vector<std::vector<Vec3> > contours = polygons[i]->getContours3D();

      Vec4 p1 = proj * makeVector4f( contours[0][1] );
      Vec4 p2 = proj * makeVector4f( contours[0][2] );
      Vec4 p3 = proj * makeVector4f( contours[0][0] );

      p1 /= p1(3);
      p2 /= p2(3);
      p3 /= p3(3);
      Vec3 a = p2.head<3>() - p1.head<3>();
      Vec3 b = p3.head<3>() - p1.head<3>();
      Vec3 normal = (a.cross(b)).normalized();
      float d = p1.head<3>().dot(normal);
      spf.addPolygon(id, normal, d);

      for(int c=0; c<contours.size(); ++c)
      {
        std::vector<Vec3>* ptr_c = &contours[c];
        projected = proj * makeVector4f( (*ptr_c)[ ptr_c->size()-1 ] );
        float inv_w = 1.0f / projected(3);
        xprev = projected(0) * inv_w;
        yprev = projected(1) * inv_w;

        for(int p=0; p<ptr_c->size(); ++p)
        {
          projected = proj * makeVector4f((*ptr_c)[p]);
          inv_w = 1.0f / projected(3);
          xcurr = projected(0) * inv_w;
          ycurr = projected(1) * inv_w;
          spf.addEdge(id, xprev, yprev, xcurr, ycurr);
          xprev = xcurr;
          yprev = ycurr;
        }
      }
    }
    // rasterize remaining polygons using scanline rendering algorithm
    spf.fill(projection);
  }
}

#endif
