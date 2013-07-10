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
 * ROS stack name: cob_environment_perception_intern
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
//internal include
#include "cob_3d_mapping_semantics/supporting_plane_extraction.h"
#include <ros/console.h>

using namespace cob_3d_mapping;

bool
SupportingPlaneExtraction::getSupportingPlane(std::vector<Polygon::Ptr>& polys, Polygon& sp)
{
  if(polys.size() == 0)
  {
    ROS_ERROR("Input polygons not set, aborting...");
    return false;
  }
  std::vector<Polygon::Ptr> cands;
  for( unsigned int i=0; i<polys.size(); i++)
  {
    double a = polys[i]->computeArea3d();
    std::cout << "Poly " << i << " (id " << polys[i]->id_ << " has an area of " << a << std::endl;
    if( a > area_min_ && a < area_max_)
    {
      std::cout << "\tadding" << std::endl;
      cands.push_back(polys[i]);
    }
  }
  if( cands.size() == 0) return false;
  int index = -1;
  double dist_min = distance_max_;
  for( unsigned int i=0; i<cands.size(); i++)
  {
    double d = cands[i]->computeDistanceFromViewpoint();
    std::cout << "Cand " << i << " has a distance of " << d << std::endl;
    if(d < distance_max_ && d > distance_min_ && d < dist_min)
    {
      dist_min = d;
      index = i;
    }
  }
  if( index == -1) return false;
  //TODO: check distance
  std::cout << cands[index]->computeArea3d() << ", " << cands[index]->computeDistanceFromViewpoint() << std::endl;
  sp = *cands[index];
  return true;//polys[index]->id;
}




