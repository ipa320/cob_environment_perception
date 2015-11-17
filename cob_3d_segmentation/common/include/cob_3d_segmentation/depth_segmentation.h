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
 *  ROS package name: cob_3d_segmentation
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

#ifndef __DEPTH_SEGMENTATION_H__
#define __DEPTH_SEGMENTATION_H__

#include <math.h>
#include <queue>

#include <ros/console.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_segmentation/general_segmentation.h"
#include "cob_3d_segmentation/cluster_graph_structure.h"

namespace cob_3d_segmentation
{
  struct SegmentationCandidate
  {
    typedef boost::shared_ptr<SegmentationCandidate> Ptr;
    SegmentationCandidate(int u_in, int v_in, float angle_in) : u(u_in), v(v_in), dot_value(angle_in) { }
    int u;
    int v;
    float dot_value;
  };

  inline const bool operator< (const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return lhs.dot_value < rhs.dot_value;}
  inline const bool operator> (const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return  operator< (rhs, lhs);}
  inline const bool operator<=(const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return !operator> (lhs, rhs);}
  inline const bool operator>=(const SegmentationCandidate& lhs, const SegmentationCandidate& rhs){return !operator< (lhs, rhs);}

  struct ptr_deref
  {
    template<typename T>
    bool operator() (const boost::shared_ptr<T>& lhs, const boost::shared_ptr<T>& rhs) const { return operator< (*lhs, *rhs); }
  };

  typedef std::priority_queue<SegmentationCandidate::Ptr, std::vector<SegmentationCandidate::Ptr>, ptr_deref> SegmentationQueue;

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
    DepthSegmentation ()
      //: max_angle_cos_(35.0f / 180.0f * M_PI)
      //, max_boundary_angle_cos_(50.0f / 180.0f * M_PI)
      //: min_dot_normals_(cos(22.0f / 180.0f * M_PI))
      : min_dot_normals_(35.0f / 180.0f * M_PI)
      , min_dot_boundary_(cos(50.0f / 180.0f * M_PI))
      //, min_cluster_size_(5)
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

    void getPotentialObjects(std::map<int,int>& objs, int max_size = 0);

    /// convert to ROS message
    virtual operator cob_3d_mapping_msgs::ShapeArray() const {ROS_ERROR("TODO: do it"); return cob_3d_mapping_msgs::ShapeArray();}

    void setMinClusterSize(const int min_cluster_size) { min_cluster_size_ = min_cluster_size; }
    int getMinClusterSize() { return min_cluster_size_; }

  private:
    void addIfIsValid(int u, int v, int idx, int idx_prev, float dist_th, float p_z, Eigen::Vector3f& n,
                      SegmentationQueue& seg_queue, ClusterPtr c);

    void computeBoundaryProperties(ClusterPtr c, EdgePtr e);
    void computeBoundaryProperties(ClusterPtr c);
    void computeEdgeSmoothness(EdgePtr e);
    void computeEdgeSmoothness();
    void addSmallNeighbors(ClusterPtr c, std::map<int,int>& objs, std::set<int>& processed, int obj_counter, int max_size);

    //float max_angle_cos_; // between mean normal of cluster and point candidates of initial segmentation
    //float max_boundary_angle_;
    float min_dot_normals_; // minimum dot product value for region growing (0 -> perpendicular, 1 -> parallel)
    float min_dot_boundary_; // minimum dot product value for boundary smoothness (0 -> perpendicular, 1 -> parallel)
    int min_cluster_size_;

    ClusterGraphPtr graph_;
    PointCloudConstPtr surface_;
    NormalCloudConstPtr normals_;
    LabelCloudPtr labels_;

  };
}

#endif
