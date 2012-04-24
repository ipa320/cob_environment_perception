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

#include <pcl/common/eigen.h>

#include "cob_3d_mapping_common/label_defines.h"


namespace cob_3d_mapping_features
{
  class Cluster
  {
  public:
    Cluster (int id_=0) : id(id_)
      , is_save_plane(false)
      , type(I_UNDEF)
      , indices()
      , sum_points(0.0, 0.0, 0.0)
      , sum_orientation(0.0, 0.0, 0.0)
      , sum_angle(0.0)
      , max_curvature(0.0)
      , min_curvature(0.0)
    { }

    ~Cluster () { }

    void
    addNewPoint(const int idx, const Eigen::Vector3f& new_point, const Eigen::Vector3f& new_normal)
    {
      indices.push_back(idx);
      sum_points += new_point;
      sum_orientation += new_normal;
    }
    
    inline std::size_t size() const { return indices.size(); }
    inline Eigen::Vector3f getCentroid() const { return (sum_points / indices.size()); }
    inline Eigen::Vector3f getOrientation() const { return (sum_orientation / indices.size()).normalized(); }
    inline bool isConvex() const { return (max_curvature < 0); }
    inline float getNormalChange() const { return sum_angle / indices.size(); }
    inline float getSurfaceCurvature() const
    { return pca_point_values(0) / (pca_point_values(0) + pca_point_values(1) + pca_point_values(2)); }


    // Members:
    const int id;
    bool is_save_plane;
    int type;
    std::vector<int> indices;
    Eigen::Vector3f sum_points;
    Eigen::Vector3f sum_orientation;
    float sum_angle;

    // Curvature Properties:
    float max_curvature;
    float min_curvature;
    Eigen::Vector3f min_curvature_direction;

    // Point Statistical Properties:
    Eigen::Vector3f pca_point_comp1;
    Eigen::Vector3f pca_point_comp2;
    Eigen::Vector3f pca_point_comp3;
    Eigen::Vector3f pca_point_values;

    // Normal Intersection Properties:
    Eigen::Vector3f pca_inter_centroid;
    Eigen::Vector3f pca_inter_comp1;
    Eigen::Vector3f pca_inter_comp2;
    Eigen::Vector3f pca_inter_comp3;
    Eigen::Vector3f pca_inter_values;

  };

  inline const bool operator< (const Cluster& lhs, const Cluster& rhs){return lhs.indices.size() < rhs.indices.size();}
  inline const bool operator> (const Cluster& lhs, const Cluster& rhs){return  operator< (rhs, lhs);}
  inline const bool operator<=(const Cluster& lhs, const Cluster& rhs){return !operator> (lhs, rhs);}
  inline const bool operator>=(const Cluster& lhs, const Cluster& rhs){return !operator< (lhs, rhs);}


  struct BoundaryPoint
  {
    Eigen::Vector3f normal;
  };

  // represents connection between two clusters:
  struct Edge 
  {
    Edge() : width(1)
      , angle(std::numeric_limits<float>::quiet_NaN())
      , d_size(0)
      , boundary_points()
      , smoothness(0.0) { }
    
    int width;
    float angle;
    int d_size;
    std::map<int,std::map<int,int> > boundary_points;
    float smoothness;
  };

  // TODO: provide a static function to recompute cluster properties after merging
  //       IN: cluster&, PointCloud::ConstPtr
}

#endif //__COB_3D_MAPPING_COMMON_CLUSTER_H__
