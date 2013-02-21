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

#ifndef __FOV_SEGMENTATION_H__
#define __FOV_SEGMENTATION_H__

//##################
//#### includes ####

/*#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>*/
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_fov_segmentation/field_of_view.h>

namespace cob_3d_mapping
{

  //####################
  //#### class ####
  class FOVSegmentation
  {
  public:
    typedef std::vector<Polygon::Ptr>::iterator polygon_iterator;
    // Constructor
    FOVSegmentation();


    // Destructor
    ~FOVSegmentation()
    {
      /// void
    }

    void setFOV(FieldOfView& fov) {
      fov_ = fov;
    }

    void setShapeArray(std::vector<Polygon::Ptr>& polygons)
    {
      polygons_in_ = polygons;
    }


    void compute(std::vector<Polygon::Ptr>& polygons);



  protected:
    void clipFOVandPlane(Polygon::Ptr& poly, std::vector<Eigen::Vector3f>& intersections);
    void clipPolygons();

    std::vector<Polygon::Ptr> polygons_in_;
    std::vector<Polygon::Ptr> polygons_fov_;
    FieldOfView fov_;

  };


} //namespace

#endif //__FOV_SEGMENTATION_H__

