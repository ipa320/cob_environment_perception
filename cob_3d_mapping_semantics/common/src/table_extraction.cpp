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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_semantics
 *
 * \author
 *  Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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
//internal include
#include "cob_3d_mapping_semantics/table_extraction.h"
#include <ros/console.h>

using namespace cob_3d_mapping;

bool
TableExtraction::isTable()
{
  if(!poly_ptr_)
  {
    ROS_ERROR("Input polygon not set, aborting...");
    return false;
  }
  //Check if the plane spanned by the polygon is horizontal or not
  if (isHorizontal() && isHeightOk() && isSizeOk())
    return true;
  else
    return false;
}

bool
TableExtraction::isHorizontal ()
{
  //check components of normal_ vector with threshold values
  if ((poly_ptr_->normal_ (2) >= norm_z_max_ || poly_ptr_->normal_ (2) <= norm_z_min_) && (poly_ptr_->normal_ (0) < norm_x_max_
      && poly_ptr_->normal_ (0) > norm_x_min_) && (poly_ptr_->normal_ (1) < norm_y_max_ && poly_ptr_->normal_ (1) > norm_y_min_))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool
TableExtraction::isHeightOk ()
{
  if(poly_ptr_->pose_.translation()(2) > height_min_ && poly_ptr_->pose_.translation()(2) < height_max_)
    return true;
  else return false;
}


bool
TableExtraction::isSizeOk ()
{
  double area = poly_ptr_->computeArea3d();
  if (area >= area_min_ && area <= area_max_)
    return true;
  return false;
}



/*void
TableExtraction::calcPolyCentroid (Polygon::Ptr poly_ptr_)
{
  //static int ctr = 0;
  double x0, y0, z0, xi, yi, zi, xi_1, yi_1, zi_1;

  double sum1, sum2, sum3, area;

  pcl::PointXYZ c_tri;
  pcl::PointXYZ cross_p;
  double tri_area;
  double mag_cross_p;

  pose_.translation()_.resize (poly_ptr_->poly_points.size ());
  for (unsigned int i = 0; i < poly_ptr_->poly_points.size (); i++)
  {
    //initialize variables
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    area = 0;

    pcl::PointXYZ v1, v2;
    x0 = poly_ptr_->poly_points[i][0][0];
    y0 = poly_ptr_->poly_points[i][0][1];
    z0 = poly_ptr_->poly_points[i][0][2];
    for (unsigned int j = 1; j < (poly_ptr_->poly_points[i].size () - 1); j++)
    {
      v1.x = 0;
      v1.y = 0;
      v1.z = 0;
      v2.x = 0;
      v2.y = 0;
      v2.z = 0;

      mag_cross_p = 0;
      tri_area = 0;
      xi = poly_ptr_->poly_points[i][j][0];
      yi = poly_ptr_->poly_points[i][j][1];
      zi = poly_ptr_->poly_points[i][j][2];

      xi_1 = poly_ptr_->poly_points[i][j + 1][0];
      yi_1 = poly_ptr_->poly_points[i][j + 1][1];
      zi_1 = poly_ptr_->poly_points[i][j + 1][2];

      v1.x = xi - x0;
      v1.y = yi - y0;
      v1.z = zi - z0;

      v2.x = xi_1 - x0;
      v2.y = yi_1 - y0;
      v2.z = zi_1 - z0;

      cross_p.x = ((v1.y * v2.z) - (v2.y * v1.z));
      cross_p.y = -((v1.x * v2.z) - (v1.z * v2.x));
      cross_p.z = ((v1.x * v2.y) - (v1.y * v2.x));

      mag_cross_p = sqrt (pow (cross_p.x, 2) + pow (cross_p.y, 2) + pow (cross_p.z, 2));
      tri_area = mag_cross_p / 2;

      c_tri.x = (x0 + xi + xi_1) / 3;
      c_tri.y = (y0 + yi + yi_1) / 3;
      c_tri.z = (z0 + zi + zi_1) / 3;

      sum1 = sum1 + (c_tri.x * tri_area);
      sum2 = sum2 + (c_tri.y * tri_area);
      sum3 = sum3 + (c_tri.z * tri_area);
      area = area + tri_area;

    }
    pose_.translation()_[i].x = (sum1 / area);
    pose_.translation()_[i].y = (sum2 / area);
    pose_.translation()_[i].z = (sum3 / area);
    poly_ptr_->pose_.translation() = pose_.translation()_;

    std::cout << "\n\t*** Centroid of polygon ( " << i << " ) " << std::endl;
    std::cout << "\tCx:" << pose_.translation()_[i].x;
    std::cout << "\tCy:" << pose_.translation()_[i].y;
    std::cout << "\tCz:" << pose_.translation()_[i].z << std::endl;
    std::cout << "\n\t*** Area of polygon ( " << i << " ) = "<< area << std::endl;

  }

}*/


