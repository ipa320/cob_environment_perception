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
 *  ROS package name: cob_3d_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 04/2012
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

#ifndef __COB_3D_SEGMENTATION_CLUSTER_HANDLER_H__
#define __COB_3D_SEGMENTATION_CLUSTER_HANDLER_H__

#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/label_defines.h>
#include "cob_3d_segmentation/cluster_types.h"

#include <list>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>


namespace cob_3d_segmentation
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

      for (size_t i=0; i<2048; ++i)
      {
        int r = (float)rand() * rand_max_inv * 255;
        int g = (float)rand() * rand_max_inv * 255;
        int b = (float)rand() * rand_max_inv * 255;
        color_tab_.push_back( r << 16 | g << 8 | b );
      }
    }
    virtual ~ClusterHandlerBase() { };

    inline ClusterPtr begin() { return clusters_.begin(); }
    inline ClusterPtr end() { return clusters_.end(); }
    inline reverse_iterator rbegin() { return clusters_.rbegin(); }
    inline reverse_iterator rend() { return clusters_.rend(); }
    inline std::pair<ClusterPtr,ClusterPtr> getClusters()
    { return std::make_pair(clusters_.begin(),clusters_.end()); }
    inline std::pair<reverse_iterator, reverse_iterator> getClustersReverse()
    { return std::make_pair(clusters_.rbegin(), clusters_.rend()); }

    inline size_type numClusters() const { return clusters_.size(); }
    inline void sortBySize() { clusters_.sort(); }
    virtual void clear() { clusters_.clear(); id_to_cluster_.clear(); max_cid_ = 0; }
    virtual void erase(ClusterPtr c) { clusters_.erase(c); }

    inline ClusterPtr getCluster(const int id)
    {
      return ( (id_to_cluster_.find(id) == id_to_cluster_.end())
               ? clusters_.end()
               : id_to_cluster_.find(id)->second );
    }

    inline ClusterPtr createCluster(int id = 0)
    {
      clusters_.push_back(ClusterType( (id<=max_cid_ ? ++max_cid_ : max_cid_ = id) ));
      return (id_to_cluster_[max_cid_] = --clusters_.end());
    }

    std::string colorHumanReadable(int id)
    {
      std::stringstream ss;
      ss << "0x" << std::setfill('0') << std::setw(6) << std::right
         << std::hex << id << std::dec;
      return ss.str();
    }

    void mapClusterColor(pcl::PointCloud<PointXYZRGB>::Ptr color_cloud)
    {
      uint32_t rgb; int t = 0;
      for(reverse_iterator c = clusters_.rbegin(); c != clusters_.rend(); ++c, ++t)
      {
        if (c->id() == I_NAN) { rgb = LabelColorMap::m.at(I_NAN); }
        else if (c->id() == I_BORDER) { rgb = LabelColorMap::m.at(I_BORDER); }
        else { rgb = color_tab_[t]; }
        for(typename ClusterType::iterator it = c->begin(); it != c->end(); ++it)
        { color_cloud->points[*it].rgb = *reinterpret_cast<float*>(&rgb); }
        c->label_color = rgb;
      }
    }

    void mapTypeColor(pcl::PointCloud<PointXYZRGB>::Ptr color_cloud)
    {
      uint32_t rgb;
      for(ClusterPtr c = clusters_.begin(); c != clusters_.end(); ++c)
      {
        rgb = LabelColorMap::m.at(c->type);
        for (typename ClusterType::iterator it = c->begin(); it != c->end(); ++it)
          color_cloud->points[*it].rgb = *reinterpret_cast<float*>(&rgb);
      }
    }

    virtual void addPoint(ClusterPtr c, int idx) = 0;
    virtual void merge(ClusterPtr source, ClusterPtr target) = 0;

  protected:
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
      c->sum_rgb_(0) += surface_->points[idx].r;
      c->sum_rgb_(1) += surface_->points[idx].g;
      c->sum_rgb_(2) += surface_->points[idx].b;
      c->color_.addColor(surface_->points[idx].r,
                         surface_->points[idx].g,
                         surface_->points[idx].b);
    }

    inline void updateNormal(ClusterPtr c, const Eigen::Vector3f& normal) const
    {
      c->sum_orientations_ += normal;
    }

    inline void clearOrientation(ClusterPtr c) const
    {
      c->sum_orientations_ = Eigen::Vector3f::Zero();
    }

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

    void addBorderIndicesToClusters()
    {
      int w = labels_->width;
      int mask[] = { -w, 1, w, -1 };
      int curr_label, count;

      for (size_t y = w; y < labels_->size() - w; y+=w)
      {
        //std::cout << "adding border points for cluster " << (*labels_)[y].label << std::endl;
        for (size_t i=y+1; i < y+w-1; ++i)
        {
          curr_label = (*labels_)[i].label;
          if(curr_label == I_NAN) continue;
          count = 0;
          for (int m=0; m<4; ++m) count += (curr_label != (*labels_)[ i+mask[m] ].label);
          if(count >= 4 || count < 1) continue;
          id_to_cluster_[curr_label]->border_points.push_back(PolygonPoint(i%w,i/w));
        }
      }
    }

    void mapClusterBorders(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
    {
      //uint32_t color = LBL_BORDER;
      for (ClusterPtr c = clusters_.begin(); c != clusters_.end(); ++c)
      {
        for(std::vector<PolygonPoint>::iterator bp = c->border_points.begin();
            bp != c->border_points.end(); ++bp)
        {
          points->points[bp->y*points->width+bp->x].rgb = LBL_BORDER;
        }
      }
    }

    void mapClusterNormalIntersectionResults(
      pcl::PointCloud<pcl::PointXYZ>::Ptr ints_centroids,
      pcl::PointCloud<pcl::Normal>::Ptr comp1,
      pcl::PointCloud<pcl::Normal>::Ptr comp2,
      pcl::PointCloud<pcl::Normal>::Ptr comp3,
      pcl::PointCloud<pcl::PointXYZ>::Ptr centroids,
      pcl::PointCloud<pcl::Normal>::Ptr connection)
    {
      ints_centroids->width = comp1->width = comp2->width = comp3->width
        = centroids->width = connection->width = numClusters();
      ints_centroids->height = comp1->height = comp2->height = comp3->height
        = centroids->width = connection->height = 1;
      ints_centroids->resize(numClusters());
      comp1->resize(numClusters());
      comp2->resize(numClusters());
      comp3->resize(numClusters());
      centroids->resize(numClusters());
      connection->resize(numClusters());

      int i = 0;
      for (ClusterPtr c = clusters_.begin(); c != clusters_.end(); ++c)
      {
        if (c->type != I_CYL) continue;
        ints_centroids->points[i].getVector3fMap() = c->pca_inter_centroid;
        comp1->points[i].getNormalVector3fMap()
          = c->pca_inter_comp1 * c->pca_inter_values(2);
        comp2->points[i].getNormalVector3fMap()
          = c->pca_inter_comp2 * c->pca_inter_values(1);
        comp3->points[i].getNormalVector3fMap()
          = c->pca_inter_comp3 * c->pca_inter_values(0);
        centroids->points[i].getVector3fMap() = c->getCentroid();
        connection->points[i].getNormalVector3fMap()
          = c->pca_inter_centroid - c->getCentroid();
        ++i;
      }
    }

  private:
    LabelCloudPtr labels_;
    PointCloudConstPtr surface_;
    NormalCloudConstPtr normals_;
  };

}

#endif
