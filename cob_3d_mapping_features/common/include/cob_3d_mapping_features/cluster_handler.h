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
 * Date of creation: 04/2012
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

#ifndef __COB_3D_MAPPING_FEATURES_CLUSTER_HANDLER_H__
#define __COB_3D_MAPPING_FEATURES_CLUSTER_HANDLER_H__

#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/label_defines.h>
#include "cob_3d_mapping_features/cluster_types.h"

#include <list>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>


namespace cob_3d_mapping_features
{
  template<typename ClusterT>
  class ClusterHandlerBase
  {
  public:
    typedef ClusterT ClusterType;
    typedef typename std::list<ClusterT>::iterator ClusterPtr;
    typedef typename std::list<ClusterT>::reverse_iterator reverse_iterator;
    typedef typename std::list<ClusterT>::size_type size_type;

  public:
    ClusterHandlerBase() : clusters_()
      , id_to_cluster_()
      , max_cid_(0)
      , color_tab_()
    {
      const float rand_max_inv = 1.0f/ RAND_MAX;
      color_tab_.resize(2048);
      color_tab_[I_UNDEF] = LBL_UNDEF;
      color_tab_[I_NAN] = LBL_NAN;
      color_tab_[I_BORDER] = LBL_BORDER;
      color_tab_[I_EDGE] = LBL_EDGE;
      color_tab_[I_PLANE] = LBL_PLANE;
      color_tab_[I_CYL] = LBL_CYL;
      color_tab_[I_SPHERE] = LBL_SPH;
      color_tab_[I_CORNER] = LBL_COR;
      for (size_t i=NUM_LABELS; i<2048 - NUM_LABELS; ++i)
      {
	int r = (float)rand() * rand_max_inv * 255;
	int g = (float)rand() * rand_max_inv * 255;
	int b = (float)rand() * rand_max_inv * 255;
	color_tab_[i] = ( r << 16 | g << 8 | b );
      }
    }
    virtual ~ClusterHandlerBase() { };

    inline ClusterPtr begin() { return clusters_.begin(); }
    inline ClusterPtr end() { return clusters_.end(); }
    inline reverse_iterator rbegin() { return clusters_.rbegin(); }
    inline reverse_iterator rend() { return clusters_.rend(); }
    inline std::pair<ClusterPtr,ClusterPtr> getClusters() { return std::make_pair(clusters_.begin(),clusters_.end()); }
    inline std::pair<reverse_iterator, reverse_iterator> getClustersReverse() 
    { return std::make_pair(clusters_.rbegin(), clusters_.rend()); }

    inline size_type numClusters() const { return clusters_.size(); }
    inline void sortBySize() { clusters_.sort(); }
    virtual void clear() { clusters_.clear(); id_to_cluster_.clear(); max_cid_ = 0; }
    virtual void erase(ClusterPtr c) { clusters_.erase(c); }

    inline ClusterPtr getCluster(const int id) 
    { return ( (id_to_cluster_.find(id) == id_to_cluster_.end()) ? clusters_.end() : id_to_cluster_.find(id)->second ); }

    inline ClusterPtr createCluster(int id = 0)
    { 
      clusters_.push_back(ClusterType( (id<=max_cid_ ? ++max_cid_ : max_cid_ = id) )); 
      return (id_to_cluster_[max_cid_] = --clusters_.end());
    }

    void mapClusterColor(pcl::PointCloud<PointXYZRGB>::Ptr color_cloud)
    {
      uint32_t rgb; int t = 3;
      for(reverse_iterator c = clusters_.rbegin(); c != clusters_.rend(); ++c, ++t)
      {
	if (c->id() == I_NAN || c->id() == I_BORDER) { rgb = color_tab_[c->id()]; --t; }
	else { rgb = color_tab_[t]; }
	for(typename ClusterType::iterator it = c->begin(); it != c->end(); ++it)
	{ color_cloud->points[*it].rgb = *reinterpret_cast<float*>(&rgb); }
      }
    }

    void mapTypeColor(pcl::PointCloud<PointXYZRGB>::Ptr color_cloud)
    {
      uint32_t rgb;
      for(ClusterPtr c = clusters_.begin(); c != clusters_.end(); ++c)
      {
	rgb = color_tab_[c->type];
	for (typename ClusterType::iterator it = c->begin(); it != c->end(); ++it)
	  color_cloud->points[*it].rgb = *reinterpret_cast<float*>(&rgb);
      }
    }

    virtual void addPoint(ClusterPtr c, int idx) = 0;
    virtual void merge(ClusterPtr source, ClusterPtr target) = 0;

  private:
    std::list<ClusterType> clusters_;
    std::map<int,ClusterPtr> id_to_cluster_;
    int max_cid_;
    std::vector<int> color_tab_;
  };


  template<typename LabelT, typename PointT, typename PointNT>
  class DepthClusterHandler : public ClusterHandlerBase<DepthCluster>
  {
  public:
    typedef boost::shared_ptr<DepthClusterHandler<LabelT,PointT,PointNT> > Ptr;

    typedef pcl::PointCloud<LabelT> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<PointNT> NormalCloud;
    typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

  public:
    DepthClusterHandler()
    { };
    ~DepthClusterHandler() { };

    void addPoint(ClusterPtr c, int idx)
    { 
      c->addIndex(idx);
      c->sum_points_ += surface_->points[idx].getVector3fMap();
      c->sum_orientations_ += normals_->points[idx].getNormalVector3fMap();
    }

    inline void updateNormal(ClusterPtr c, const Eigen::Vector3f& normal) const { c->sum_orientations_ += normal; }
    inline void clearOrientation(ClusterPtr c) const { c->sum_orientations_ = Eigen::Vector3f::Zero(); }

    inline void merge(ClusterPtr source, ClusterPtr target)
    { 
      for (ClusterType::iterator idx = source->begin(); idx != source->end(); ++idx)
      {
	addPoint(target, *idx);
	labels_->points[*idx].label = target->id();
      }
      erase(source);
    }

    inline void setLabelCloudInOut(LabelCloudPtr labels) { labels_ = labels; }
    inline void setPointCloudIn(PointCloudConstPtr points) { surface_ = points; }
    inline void setNormalCloudIn(NormalCloudConstPtr normals) { normals_ = normals; }

    void computeClusterComponents(ClusterPtr c);
    void computeCurvature(ClusterPtr c);
    void computePointCurvature(ClusterPtr c, int search_size);
    bool computePrincipalComponents(ClusterPtr c);
    void computeNormalIntersections(ClusterPtr c);
    //void computeColorHistogram(ClusterPtr c);

  private:
    LabelCloudPtr labels_;
    PointCloudConstPtr surface_;
    NormalCloudConstPtr normals_;
  };

}

#endif
