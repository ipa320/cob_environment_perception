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
 * ROS package name: cob_3d_segmentation
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

#ifndef __DEPTH_SEGMENTATION_H__
#define __DEPTH_SEGMENTATION_H__

#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_segmentation/general_segmentation.h"
#include "cob_3d_segmentation/cluster_graph_structure.h"

namespace cob_3d_segmentation
{
  struct SegmentationCandidate
  {
    SegmentationCandidate(int u_in, int v_in, float angle_in) : u(u_in), v(v_in), angle(angle_in) { }
    int u;
    int v;
    float angle;
  };

  inline const bool operator< (const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return lhs.angle < rhs.angle;}
  inline const bool operator> (const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return  operator< (rhs, lhs);}
  inline const bool operator<=(const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return !operator> (lhs, rhs);}
  inline const bool operator>=(const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return !operator< (lhs, rhs);}

  struct PredefinedSegmentationTypes
  {
    typedef PointLabel Label;
    typedef pcl::PointXYZRGB Point;
    typedef pcl::Normal Normal;
    typedef DepthClusterHandler<Label,Point,Normal> CH;
    typedef BoundaryPointsEdgeHandler<Label,Point> EH;
    typedef ClusterGraphStructure<CH,EH> Graph;
  };


  template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT>
    class DepthSegmentation : public GeneralSegmentation<PointT, PointLabelT>
  {
  public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::PointCloud<PointNT> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;
    typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

    typedef pcl::PointCloud<PointLabelT> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;
    
    typedef typename ClusterGraphT::Ptr ClusterGraphPtr;
    typedef typename ClusterGraphT::ClusterPtr ClusterPtr;
    typedef typename ClusterGraphT::EdgePtr EdgePtr;

  public:
    DepthSegmentation () : max_angle_(35.0f / 180.0f * M_PI)
      , max_boundary_angle_(50.0f / 180.0f * M_PI)
      , min_cluster_size_(150)
    { };
    
    ~DepthSegmentation ()
    { };

    inline void setClusterGraphOut(ClusterGraphPtr clusters) { graph_ = clusters; }
    
    inline void setNormalCloudIn(NormalCloudConstPtr normals) { normals_ = normals; }
    inline void setLabelCloudInOut(LabelCloudPtr labels) { labels_ = labels; }
    virtual void setInputCloud(const PointCloudConstPtr& points) { surface_ = points; }
    virtual LabelCloudConstPtr getOutputCloud() { return labels_; }
    virtual bool compute() 
    { 
      performInitialSegmentation();
      refineSegmentation();
      return true;
    }

    void performInitialSegmentation();

    void refineSegmentation();

  private:
    void addIfIsValid(int u, int v, int idx, int idx_prev, float dist_th, float p_z, Eigen::Vector3f& n,
		      std::multiset<SegmentationCandidate>& coords_todo, ClusterPtr c);

    void computeBoundaryProperties(ClusterPtr c, EdgePtr e);
    void computeBoundaryProperties(ClusterPtr c);
    void computeEdgeSmoothness(EdgePtr e);
    void computeEdgeSmoothness();

    float max_angle_; // between mean normal of cluster and point candidates of initial segmentation
    float max_boundary_angle_;
    int min_cluster_size_;

    ClusterGraphPtr graph_;
    PointCloudConstPtr surface_;
    NormalCloudConstPtr normals_;
    LabelCloudPtr labels_;
    
  };
}

#endif
