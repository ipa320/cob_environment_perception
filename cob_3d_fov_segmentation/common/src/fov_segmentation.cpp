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
 * Computes field of view segmentation of a shape array.
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


//##################
//#### includes ####

#include <cob_3d_fov_segmentation/fov_segmentation.h>

using namespace cob_3d_mapping;


// Constructor
FOVSegmentation::FOVSegmentation()
{
}

void FOVSegmentation::compute(std::vector<Polygon::Ptr>& polygons)
{
  for(polygon_iterator it = polygons_in_.begin(); it != polygons_in_.end(); it++)
  {
    std::vector<Eigen::Vector3f> intersections;
    clipFOVandPlane(*it, intersections);
    if( intersections.size() == 0) continue;
    Polygon::Ptr fov_poly(new Polygon(**it));
    fov_poly->contours.clear();
    fov_poly->holes.clear();
    fov_poly->contours.push_back(intersections);
    fov_poly->holes.push_back(false);
    (*it)->merge_difference(fov_poly);
    polygons.push_back(fov_poly);
  }
}

void FOVSegmentation::clipFOVandPlane(Polygon::Ptr& poly, std::vector<Eigen::Vector3f>& intersections)
{
  std::vector<Eigen::Vector3d> p(5);
  /*Eigen::Vector3d p_1;
  Eigen::Vector3d p_2;
  Eigen::Vector3d p_3;
  Eigen::Vector3d p_4;*/
  fov_.getFOV(p[0], p[1], p[2], p[3], p[4]);

  for( unsigned int i=1; i<p.size(); i++)
  {
    double div = poly->normal.cast<double>().dot(p[i]-p[0]);
    if ( div = 0 ) continue;
    double lambda = (poly->normal.cast<double>().dot(poly->centroid.topLeftCorner(3, 1).cast<double>() - p[0]))/div;
    if ( lambda > 1 || lambda < 0) continue;
    Eigen::Vector3d intersection = p[0] + lambda*(p[i]-p[0]);
    intersections.push_back(intersection.cast<float>());
    unsigned int j = i;
    if ( j == p.size()-1 ) j = 1;
    div = poly->normal.cast<double>().dot(p[j+1]-p[j]);
    if ( div = 0 ) continue;
    lambda = (poly->normal.cast<double>().dot(poly->centroid.topLeftCorner(3, 1).cast<double>() - p[j]))/div;
    if ( lambda > 1  || lambda < 0) continue;
    Eigen::Vector3d intersection2 = p[j] + lambda*(p[j+1]-p[j]);
    intersections.push_back(intersection2.cast<float>());
  }
}

void FOVSegmentation::clipPolygons()
{

}


