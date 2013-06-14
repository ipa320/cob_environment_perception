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
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
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

#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

#include <algorithm>

namespace cob_3d_mapping
{
  class PrimeSense
  {
    public:
    PrimeSense() {}
    ~PrimeSense() {}

    static bool areNeighbors(float query, float neighbor, float tolerance = 2.0f)
    {
      float dist_th = tolerance * 0.003f * query * query;
      //if (query < 1.2) dist_th += 0.01f;
      return (fabs(query - neighbor) < dist_th);
    }

    static bool areNeighbors(const Eigen::Vector3f& query, const Eigen::Vector3f&  neighbor, float tolerance = 4.0f)
    {
      tolerance += max( 0.0f, 1.0f * (4.0f - min(fabs(query(0)),fabs(query(1))) * 10.0f) );
      float dist_th =  tolerance * 0.003f * query(2) * query(2);
      return (fabs(query(2) - neighbor(2)) < dist_th);
    }
  };
}

#endif
