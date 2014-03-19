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
 * \date Date of creation: 05/2012
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

#ifndef __COB_3D_SEGMENTATION_EDGE_HANDLER_H__
#define __COB_3D_SEGMENTATION_EDGE_HANDLER_H__

#include <cob_3d_mapping_common/label_defines.h>
#include "cob_3d_segmentation/edge_types.h"

#include <boost/function.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cob_3d_segmentation
{
  template<typename EdgeT>
  class EdgeHandlerBase
  {
  public:
    typedef EdgeT EdgeType;
    typedef typename std::list<EdgeT>::iterator EdgePtr;
    typedef typename std::list<EdgeT>::reverse_iterator reverse_iterator;
    typedef typename std::list<EdgeT>::size_type size_type;

  public:
    EdgeHandlerBase()
    { };
    virtual ~EdgeHandlerBase() { };

    inline EdgePtr begin() { return edges_.begin(); }
    inline EdgePtr end() { return edges_.end(); }
    inline reverse_iterator rbegin() { return edges_.rbegin(); }
    inline reverse_iterator rend() { return edges_.rend(); }

    inline std::pair<EdgePtr,EdgePtr> getEdges() { return std::make_pair(edges_.begin(),edges_.end()); }
    inline size_type numEdges() const { return edges_.size(); }
    virtual void erase(EdgePtr e) { edges_.erase(e); }
    virtual void clear() { edges_.clear(); }

    inline EdgePtr createEdge() { edges_.push_back(EdgeType()); return --edges_.end(); }
    virtual void merge(EdgePtr source, EdgePtr target) = 0;
    virtual void move(int old_cid, int new_cid, EdgePtr e) = 0;

  private:
    std::list<EdgeType> edges_;
  };

  template<typename LabelT, typename PointT>
  class BoundaryPointsEdgeHandler : public EdgeHandlerBase<BoundaryPointsEdge>
  {
  public:
    typedef boost::shared_ptr<BoundaryPointsEdgeHandler<LabelT,PointT> > Ptr;

    typedef pcl::PointCloud<LabelT> LabelCloud;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  public:
    BoundaryPointsEdgeHandler()
      : edge_validator(BoundarySmoothnessValidator(0.8f))
      ,boundary_points_()
    { };

    ~BoundaryPointsEdgeHandler() { };

    void mapBoundaryPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
    {
      uint32_t color = LBL_BORDER;
      for(std::map<int,BoundaryPoint>::iterator b_it = boundary_points_.begin(); b_it != boundary_points_.end(); ++b_it)
      {
        points->points[b_it->first].rgb = *reinterpret_cast<float*>(&color);
      }
    }

    void mapBoundaryPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals)
    {
      points->clear();
      normals->clear();
      points->resize(boundary_points_.size());
      normals->resize(boundary_points_.size());
      points->width = normals->width = 1;
      points->height = normals->height = boundary_points_.size();
      pcl::PointCloud<pcl::PointXYZRGB>::iterator p_it = points->begin();
      pcl::PointCloud<pcl::Normal>::iterator n_it = normals->begin();
      for(std::map<int,BoundaryPoint>::iterator b_it = boundary_points_.begin();
          b_it != boundary_points_.end(); ++b_it, ++p_it, ++n_it)
      {
        *p_it = surface_->points[b_it->first];
        n_it->getNormalVector3fMap() = b_it->second.normal;
      }
    }

    void erase(EdgePtr e);
    void merge(EdgePtr source, EdgePtr target);
    void move(int old_cid, int new_cid, EdgePtr e);

    inline void updateProperties(EdgePtr e, const int cid1, const int idx1, const int cid2, const int idx2)
    {
      e->boundary_pairs[cid1].push_back(idx1); // idx1 is at border of cluster cid1
      e->boundary_pairs[cid2].push_back(idx2); // idx2 is at border of cluster cid2
      boundary_points_.insert(std::make_pair(idx1, BoundaryPoint(idx2))); // idx1 is BoundaryPoint with brother idx2
      boundary_points_.insert(std::make_pair(idx2, BoundaryPoint(idx1))); // idx2 is BoundaryPoint with brother idx1
    }
    inline BoundaryPoint& getBoundaryPoint(const int idx) { return boundary_points_[idx]; }

    const boost::function<bool (EdgePtr)> edge_validator;

    inline void setLabelCloudIn(LabelCloudConstPtr labels) { labels_ = labels; }
    inline void setPointCloudIn(PointCloudConstPtr points) { surface_ = points; }

  private:
    std::map<int,BoundaryPoint> boundary_points_;
    LabelCloudConstPtr labels_;
    PointCloudConstPtr surface_;

    class BoundarySmoothnessValidator
    {
    public:
      BoundarySmoothnessValidator(float min_smoothness) : min_smoothness_(min_smoothness)
      { }

      inline bool operator() (EdgePtr e) { return (e->smoothness > min_smoothness_); }
    private:
      float min_smoothness_;
    };
  };
}

#endif
