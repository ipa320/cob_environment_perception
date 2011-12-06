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
 * Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
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

#ifndef SEMANTIC_EXTRACTION_H_
#define SEMANTIC_EXTRACTION_H_

//##################
//#### includes ####

//standard includes
#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

class SemanticExtraction
{

public:
  struct Polygon
  {
    std::vector<std::vector<Eigen::Vector3f> > poly_points;
    Eigen::Vector3f normal;
    float d;
  };

  typedef boost::shared_ptr<Polygon> PolygonPtr;

  // Constructor
  SemanticExtraction ()
  {

  }

  // Destructor
  ~SemanticExtraction ()
  {
    /// void
  }

  // checks if the plane of the polygon is horizontal
  bool
  isHorizontal (PolygonPtr p_ptr);

  // checks if the plane is high enough
  bool
  isHeightOk (PolygonPtr p_ptr);

  // checks if the area of the plane is sufficient
  bool
  isSizeOk (PolygonPtr p_ptr);

  //protected:

};

#endif /* SEMANTIC_EXTRACTION_H_ */
