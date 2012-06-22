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
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
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

#ifndef __COB_3D_SEGMENTATION_CLUSTER_TYPES_H__
#define __COB_3D_SEGMENTATION_TYPES_H__

#include <pcl/common/eigen.h>
#include <set>

namespace cob_3d_segmentation
{
  class ClusterBase
  {
  public:
    typedef std::vector<int>::iterator iterator;
    typedef std::vector<int>::size_type size_type;
    typedef std::vector<int>::value_type value_type;

  public:
    explicit ClusterBase(int id)
      : id_(id)
      , indices_()
    { };
    virtual ~ClusterBase() { }

    inline const int id() const { return id_; }
    
    inline value_type& operator[](size_type idx) { return indices_.at(idx); }
    inline iterator begin() { return indices_.begin(); }
    inline iterator end() { return indices_.end(); }
    inline size_type size() const { return indices_.size(); }

    inline void addIndex(int idx) { indices_.push_back(idx); }
    
    std::vector<int> indices_;    
  protected:
    int id_;

  };

  class DepthCluster : public ClusterBase
  {
    template <typename LabelT, typename PointT, typename PointNT> friend class DepthClusterHandler;

  public:
    DepthCluster(int id)
      : ClusterBase(id)
      , type(I_UNDEF)
      , type_probability(1.0)
      , is_save_plane(false)
      , max_curvature(0.0)
      , min_curvature(0.0)
      , pca_point_comp1(0.0, 0.0, 0.0)
      , pca_point_comp2(0.0, 0.0, 0.0)
      , pca_point_comp3(0.0, 0.0, 0.0)
      , pca_point_values(0.0, 0.0, 0.0)
      , border_indices()
      , sum_points_(0.0, 0.0, 0.0)
      , sum_orientations_(0.0, 0.0, 0.0)
    { }

    inline Eigen::Vector3f getCentroid() const { return sum_points_ / static_cast<float>(indices_.size()); }
    inline Eigen::Vector3f getOrientation() const { return sum_orientations_ / static_cast<float>(indices_.size()); }

  public:
    int type;
    float type_probability;
    bool is_save_plane;
    float max_curvature;
    float min_curvature;

    Eigen::Vector3f pca_point_comp1;
    Eigen::Vector3f pca_point_comp2;
    Eigen::Vector3f pca_point_comp3;
    Eigen::Vector3f pca_point_values;

    Eigen::Vector3f pca_inter_centroid;
    Eigen::Vector3f pca_inter_comp1;
    Eigen::Vector3f pca_inter_comp2;
    Eigen::Vector3f pca_inter_comp3;
    Eigen::Vector3f pca_inter_values;

    std::vector<int> border_indices;

  private:
    Eigen::Vector3f sum_points_;
    Eigen::Vector3f sum_orientations_;
  };

  inline const bool operator< (const ClusterBase& lhs, const ClusterBase& rhs){return lhs.size() < rhs.size();}
  inline const bool operator> (const ClusterBase& lhs, const ClusterBase& rhs){return  operator< (rhs, lhs);}
  inline const bool operator<=(const ClusterBase& lhs, const ClusterBase& rhs){return !operator> (lhs, rhs);}
  inline const bool operator>=(const ClusterBase& lhs, const ClusterBase& rhs){return !operator< (lhs, rhs);}
}

#endif
