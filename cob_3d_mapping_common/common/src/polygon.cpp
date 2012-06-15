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
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
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

#include "cob_3d_mapping_common/polygon.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

namespace cob_3d_mapping
{

void
Polygon::computeCentroid()
{
  //find largest non-hole contour
  unsigned int idx = 0;
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    int max_pts = 0;
    if(!holes[i])
    {
      if(contours[i].size()>max_pts)
      {
        max_pts = contours[i].size();
        idx = i;
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ> poly_cloud;
  for (unsigned int j = 0; j < contours[idx].size () ; j++)
  {
    pcl::PointXYZ p;
    p.x = contours[idx][j][0];
    p.y = contours[idx][j][1];
    p.z = contours[idx][j][2];
    poly_cloud.push_back(p);
  }
  pcl::compute3DCentroid(poly_cloud,centroid);
}

double
Polygon::computeArea()
{
  double xi, xi_1, yi, yi_1, area=0;

  //area_.resize (poly_ptr_->contours.size ());
  double sum;
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    if(holes[i]) continue;
    sum = 0;
    //area_[i] = 0;
    for (unsigned int j = 0; j < contours[i].size (); j++)
    {
      xi = contours[i][j][0];
      yi = contours[i][j][1];
      if (j == (contours[i].size ()) - 1)
      {
        xi_1 = contours[i][0][0];
        yi_1 = contours[i][0][1];
      }
      else
      {
        xi_1 = contours[i][j + 1][0];
        yi_1 = contours[i][j + 1][1];
      }
      sum = sum + (xi * yi_1 - xi_1 * yi);
      /*
       std::cout << " ---------------------------------------" << std::endl;
       std::cout << " isSizeOk: xi-->" << xi << std::endl;
       std::cout << " \t: xi_1-->" << xi_1 << std::endl;
       std::cout << " \nisSizeOk: yi-->" << yi << std::endl;
       std::cout << " \t: yi_1-->" << yi_1 << std::endl;
       std::cout << " isSizeOk: sum-->" << sum << std::endl;
       std::cout << " ++++++++++++++++++++++++++++++++++++++++" <<std::endl;
       */

    }
    area += fabs (sum / 2);
    //std::cout << "\n\t*** Area of polygon ( " << i << " ) = " << area_[i] << std::endl;
  }
  return area;
}

}
