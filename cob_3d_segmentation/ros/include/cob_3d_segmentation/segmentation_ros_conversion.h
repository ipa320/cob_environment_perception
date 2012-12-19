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

#ifndef SEGMENTATION_ROS_CONVERSTION_H__
#define SEGMENTATION_ROS_CONVERSTION_H__

#include <cob_3d_segmentation/segmentation_conversion.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>

namespace cob_3d_segmentation
{
  template<
    typename PointT = pcl::PointXYZRGB,
    typename ClusterHdlT = DepthClusterHandler<PointLabel, PointT, pcl::Normal>
    >
  class ClusterROSConversion : public ClusterConversion<PointT, ClusterHdlT>
  {
  public:
    typedef ClusterConversion<PointT, ClusterHdlT> Base;

    using Base::min_size_;
    using Base::max_dist_;

    using Base::pe_;
    using Base::ch_;
    using Base::p_;

    bool clusterToShapeMsg(const typename Base::ClusterPtr& c, cob_3d_mapping_msgs::Shape& s)
    {
      PolygonContours<PolygonPoint> poly;
      pe_.outline(p_->width, p_->height, c->border_points, poly);
      if(!poly.polys_.size()) false; // continue, if no contours were found
      int max_idx=0, max_size=0;
      for (int i = 0; i < (int)poly.polys_.size(); ++i)
      {
        if ((int)poly.polys_[i].size() > max_size) { max_idx = i; max_size = poly.polys_[i].size(); }
      }

      Eigen::Vector3f centroid = c->getCentroid();

      s.id = 0;
      s.points.resize(poly.polys_.size());
      s.header.frame_id = p_->header.frame_id.c_str();

      typename Base::PointCloud::Ptr hull(new typename Base::PointCloud);
      for (int i = 0; i < (int)poly.polys_.size(); ++i)
      {
        if (i == max_idx)
        {
          s.holes.push_back(false);
          std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin();
          for ( ; it != poly.polys_[i].end(); ++it) {
            hull->push_back( p_->points[ it->x + it->y * p_->width ] );
          }
        }
        else
        {
          s.holes.push_back(true);
          std::vector<PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin();
          for ( ; it != poly.polys_[i].rend(); ++it) {
            hull->push_back( p_->points[ it->x + it->y * p_->width ] );
          }
        }
        hull->height = 1;
        hull->width = hull->size();
        pcl::toROSMsg(*hull, s.points[i]);
        hull->clear();
      }

      s.centroid.x = centroid[0];
      s.centroid.y = centroid[1];
      s.centroid.z = centroid[2];
      s.type = cob_3d_mapping_msgs::Shape::POLYGON;
      s.params.resize(4);
      s.params[0] = c->pca_point_comp3(0);
      s.params[1] = c->pca_point_comp3(1);
      s.params[2] = c->pca_point_comp3(2);
      s.params[3] = fabs(centroid.dot(c->pca_point_comp3)); // d
      return true;
    }

    void convertToShapeArray(cob_3d_mapping_msgs::ShapeArray& sa)
    {
      sa.header = p_->header;
      sa.header.frame_id = p_->header.frame_id.c_str();

      for(typename Base::ClusterPtr c = ch_->begin(); c != ch_->end(); ++c)
      {
        if(c->size() < min_size_) continue;
        if(c->getCentroid()(2) > max_dist_) continue;
        if(c->size() <= ceil(1.1f * (float)c->border_points.size())) continue;

        ch_->computeClusterComponents(c);
        if(!c->is_save_plane) continue;

        sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
        if(!clusterToShapeMsg(c, sa.shapes.back())) {
          sa.shapes.pop_back();
        }
      }
    }
  };
}

#endif
