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
 *  ROS package name: cob_3d_mapping_pipeline
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2012
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

#ifndef CLUSTER_CONVERSION_H__
#define CLUSTER_CONVERSION_H__

#include <pcl/point_types.h>

#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>
#include <cob_3d_segmentation/cluster_handler.h>

namespace cob_3d_segmentation
{
  template<
    typename PointT = pcl::PointXYZRGB,
    typename ClusterHdlT = DepthClusterHandler<PointLabel, PointT, pcl::Normal>
  >
  class ClusterConversion
  {
    public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename ClusterHdlT::Ptr ClusterHdlPtr;
    typedef typename ClusterHdlT::ClusterPtr ClusterPtr;

    public:
    ClusterConversion()
    : min_size_(100)
    , max_dist_(5.0f)
    {}

    virtual ~ClusterConversion() {}

    inline void setClusterHandler(const ClusterHdlPtr& ch) { ch_ = ch; }
    inline void setInputCloud(const PointCloudConstPtr& p) { p_ = p; }
    inline void setMinClusterSize(int size) { min_size_ = size; }
    inline void setMaxCentroidDistance(float z_distance) { max_dist_ = z_distance; }

    bool clusterToPolygon(const ClusterPtr& c, cob_3d_mapping::Polygon& pg)
    {
      cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> poly;
      pe_.outline(p_->width,p_->height,c->border_points,poly);
      if(!poly.polys_.size()) return false;
      int max_idx=0, max_size=0;
      for (int i=0; i<(int)poly.polys_.size(); ++i)
      {
        if( (int)poly.polys_[i].size() > max_size) {
          max_idx = i;
          max_size = poly.polys_[i].size();
        }
      }

      Eigen::Vector3f centroid = c->getCentroid();

      pg.id = c->id();
      Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - c->pca_point_comp3 * c->pca_point_comp3.transpose();
      for(int i=0; i<(int)poly.polys_.size(); ++i)
      {
        if (i==max_idx)
        {
          pg.holes.push_back(false);
          pg.contours.push_back(std::vector<Eigen::Vector3f>());
          std::vector<cob_3d_segmentation::PolygonPoint>::iterator it = poly.polys_[i].begin();
          for( ; it!=poly.polys_[i].end(); ++it)
          {
            pg.contours.back().push_back( M * ( (*p_)[it->x + it->y * p_->width].getVector3fMap() - centroid ) );
            pg.contours.back().back() += centroid;
          }
        }
        else
        {
          pg.holes.push_back(true);
          pg.contours.push_back(std::vector<Eigen::Vector3f>());
          std::vector<cob_3d_segmentation::PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin();
          for( ; it!=poly.polys_[i].rend(); ++it)
          {
            pg.contours.back().push_back( M * ( (*p_)[it->x + it->y * p_->width].getVector3fMap() - centroid ) );
            pg.contours.back().back() += centroid;
          }
        }
      }
      pg.centroid << centroid(0), centroid(1), centroid(2), 1.0f;
      pg.normal = c->pca_point_comp3;
      pg.d = fabs(centroid.dot(c->pca_point_comp3));
      return true;
    }

    void convertToPolygons(std::vector<cob_3d_mapping::Polygon::Ptr> polygons)
    {
      for(ClusterPtr c = ch_->begin(); c != ch_->end(); ++c)
      {
        if(c->size() < min_size_) continue;
        if(c->getCentroid()(2) > max_dist_) continue;
        if(c->size() <= ceil(1.1f * (float)c->border_points.size())) continue;

        ch_->computeClusterComponents(c);
        if(!c->is_save_plane) continue;

        cob_3d_mapping::Polygon::Ptr p(new cob_3d_mapping::Polygon);
        if(clusterToPolygon(c, *p)) {
          polygons.push_back(p);
        }
      }
    }

    protected:
    int min_size_;
    float max_dist_;

    PolygonExtraction pe_;
    ClusterHdlPtr ch_;
    PointCloudConstPtr p_;
  };
}

#endif
