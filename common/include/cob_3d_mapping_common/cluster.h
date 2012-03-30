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
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
 * ToDo:
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

#ifndef __COB_3D_MAPPING_COMMON_CLUSTER_H__
#define __COB_3D_MAPPING_COMMON_CLUSTER_H__

#include "cob_3d_mapping_common/label_defines.h"

namespace cob_3d_mapping_common
{
  class Cluster
  {
  public:
    Cluster () : indices()
      , type(I_UNDEF)
      , sum_points(0.0, 0.0, 0.0)
      , sum_orientation(0.0, 0.0, 0.0)
      , sum_angle(0.0)
      , sum_max_curvature(0.0)
      , sum_min_curvature(0.0)
    { };

    ~Cluster () { };

    void
      updateCluster(const int idx, 
		    const Eigen::Vector3f& new_point, 
		    const Eigen::Vector3f& new_normal)
    {
      indices.push_back(idx);
      sum_points += new_point;
      sum_orientation += new_normal;
    }

    void
    updateCluster(const int idx,
		  const Eigen::Vector3f& new_normal, 
		  const float c_max,
		  const float c_min)
    {
      indices.push_back(idx);
      sum_orientation += new_normal;

      sum_max_curvature += c_max;
      sum_min_curvature += c_min;
    }

    inline Eigen::Vector3f
    getCentroid() { return (sum_points / indices.size()); }
    
    inline Eigen::Vector3f
      getOrientation() { return (sum_orientation / indices.size()).normalized(); }

    inline float
    getNormalChange() { return sum_angle / indices.size(); }

    inline float
    getSurfaceCurvature() { return eigenvalues(0) / (eigenvalues(0) + eigenvalues(1) + eigenvalues(2)); }

    inline float
    getMaxCurvature() { return (sum_max_curvature / indices.size()); }

    inline float
    getMinCurvature() { return (sum_min_curvature / indices.size()); }

    inline bool
    isConvex() { return (sum_max_curvature < 0); }

    int type;
    std::vector<int> indices;
    Eigen::Vector3f first_component;
    Eigen::Vector3f second_component;
    Eigen::Vector3f third_component;
    Eigen::Vector3f eigenvalues;

  protected:
    Eigen::Vector3f sum_points;
    Eigen::Vector3f sum_orientation;
    float sum_angle;
    float sum_max_curvature;
    float sum_min_curvature;

  };

  inline bool operator< (const Cluster& lhs, const Cluster& rhs){return lhs.indices.size() < rhs.indices.size();}
  inline bool operator> (const Cluster& lhs, const Cluster& rhs){return  operator< (rhs, lhs);}
  inline bool operator<=(const Cluster& lhs, const Cluster& rhs){return !operator> (lhs, rhs);}
  inline bool operator>=(const Cluster& lhs, const Cluster& rhs){return !operator< (lhs, rhs);}
}

#endif //__COB_3D_MAPPING_COMMON_CLUSTER_H__
