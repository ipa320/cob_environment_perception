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

#ifndef __IMPL_DEPTH_SEGMENTATION_HPP__
#define __IMPL_DEPTH_SEGMENTATION_HPP__

#include <set>

#include <pcl/common/eigen.h>
#include <Eigen/LU>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_segmentation/depth_segmentation.h"
#include "cob_3d_features/organized_normal_estimation.h"

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::addIfIsValid(
  int u, int v, int idx, int idx_prev, float dist_th, float p_z, Eigen::Vector3f& n,
  SegmentationQueue& seg_queue, ClusterPtr c)
{
  if (p_z < 1.2)
  {
    if (fabs(p_z - surface_->points[idx].z) > (dist_th + 0.01f)) return;
  }
  else if (fabs(p_z - surface_->points[idx].z) > dist_th) return;
  int* p_label = &(labels_->points[idx].label);
  if (*p_label == I_UNDEF)
  {

    /*float angle = fabs( atan2((n.cross(normals_->points[idx].getNormalVector3fMap())).norm(),
      n.dot(normals_->points[idx].getNormalVector3fMap())) );*/
    float dot_value = fabs( n.dot(normals_->points[idx].getNormalVector3fMap()) );
    if ( (int)c->size() < min_cluster_size_ || dot_value > min_dot_normals_)
    {
      *p_label = c->id();
      seg_queue.push( SegmentationCandidate::Ptr(new SegmentationCandidate(u, v, dot_value)) );
    }
  }
  else if (*p_label > I_EDGE && c->id() != *p_label)
  {
    graph_->edges()->updateProperties(graph_->connect(c->id(), *p_label), c->id(), idx_prev, *p_label, idx);
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::performInitialSegmentation()
{
  int width = labels_->width, height = labels_->height;
  // reset graph and property handlers:
  graph_->clear();
  graph_->clusters()->setLabelCloudInOut(labels_);
  graph_->clusters()->setPointCloudIn(surface_);
  graph_->clusters()->setNormalCloudIn(normals_);
  graph_->edges()->setLabelCloudIn(labels_);
  graph_->edges()->setPointCloudIn(surface_);
  graph_->clusters()->createCluster(I_NAN)->type = I_NAN;
  graph_->clusters()->createCluster(I_BORDER)->type = I_BORDER;
  graph_->clusters()->createCluster(I_EDGE)->type = I_EDGE;

  // fire!
  for(size_t i=0; i<labels_->points.size(); ++i)
  {
    if (labels_->points[i].label == I_UNDEF)
    {
      ClusterPtr c = graph_->clusters()->createCluster();
      //std::multiset<SegmentationCandidate> seg_queue;
      SegmentationQueue seg_queue;
      seg_queue.push( SegmentationCandidate::Ptr(new SegmentationCandidate(i%width, i/width, 0.0)) );
      labels_->points[i].label = c->id();
      while (seg_queue.size() != 0)
      {
        SegmentationCandidate p = *seg_queue.top(); // get point with max dot_value!
        seg_queue.pop();
        int p_idx = p.v * width + p.u;
        float p_z = surface_->points[p_idx].z;
        float dist_th = 2.0 * 0.003 * p_z * p_z;
        graph_->clusters()->addPoint(c, p_idx); // clusterhandler takes care of properties

        Eigen::Vector3f n_n = c->getOrientation();
        // Look right
        if (p.u+1 < width) { addIfIsValid(p.u+1, p.v, p_idx+1, p_idx, dist_th, p_z, n_n, seg_queue, c); }
        // Look down
        if (p.v+1 < height) { addIfIsValid(p.u, p.v+1, p_idx+width, p_idx, dist_th, p_z, n_n, seg_queue, c); }
        // Look left
        if (p.u > 0) { addIfIsValid(p.u-1, p.v, p_idx-1, p_idx, dist_th, p_z, n_n, seg_queue, c); }
        // Look up
        if (p.v > 0) { addIfIsValid(p.u, p.v-1, p_idx-width, p_idx, dist_th, p_z, n_n, seg_queue, c); }
      } // end while

      // merge small clusters
      if (c->size() < min_cluster_size_)
      {
        std::vector<ClusterPtr> adj_list;
        graph_->getAdjacentClusters(c->id(), adj_list);
        if (adj_list.size() != 0)
        {
          int max_cluster_id = adj_list.front()->id();
          int max_edge_width = 0;
          for (typename std::vector<ClusterPtr>::iterator it = adj_list.begin(); it != adj_list.end(); ++it)
          {
            EdgePtr e = graph_->getConnection(c->id(), (*it)->id());
            if (e->size() > max_edge_width) { max_cluster_id = (*it)->id(); max_edge_width = e->size(); }
          }
          graph_->merge( c->id(), max_cluster_id );
        }
      }
    }
    else if(labels_->points[i].label <= I_EDGE)
    {
      graph_->clusters()->getCluster(labels_->points[i].label)->addIndex(i);
    }
    else
      continue;
  }
  return;
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::refineSegmentation()
{
  graph_->clusters()->addBorderIndicesToClusters();
  graph_->clusters()->sortBySize(); // Ascending order
  ClusterPtr c_it, c_end;
  for (boost::tie(c_it,c_end) = graph_->clusters()->getClusters(); c_it != c_end; ++c_it)
  {
    //std::cout << c_it->size() << std::endl;
    if (c_it->size() < 20) continue;
    if (c_it->type == I_EDGE || c_it->type == I_NAN || c_it->type == I_BORDER) continue;
    computeBoundaryProperties(c_it);
  }
  computeEdgeSmoothness();
  for ( --c_end; c_end != graph_->clusters()->begin(); --c_end) // iterate from back to front
  {
    if ( c_end->size() < 5 ) continue;
    if ( c_end->type == I_EDGE || c_end->type == I_NAN || c_end->type == I_BORDER) continue;
    std::vector<ClusterPtr> adj_list;
    //graph_->getConnectedClusters(c_end->id(), adj_list, graph_->edges()->edge_validator);
    graph_->getAdjacentClusters(c_end->id(), adj_list);
    for (typename std::vector<ClusterPtr>::iterator a_it = adj_list.begin(); a_it != adj_list.end(); ++a_it)
    {
      EdgePtr e = graph_->getConnection(c_end->id(), (*a_it)->id());
      if ( e->smoothness < 0.8 ) continue;
      //std::cout << sqrt((*a_it)->size()) * 1.0f << " > " << e->size() << std::endl;
      if ( e->size() < sqrt((*a_it)->size()) * 0.6f ) continue;
      std::vector<EdgePtr> updated_edges;
      graph_->merge( (*a_it)->id(), c_end->id(), updated_edges );
      for (typename std::vector<EdgePtr>::iterator e_it = updated_edges.begin(); e_it != updated_edges.end(); ++e_it)
      {
        computeBoundaryProperties(c_end, *e_it);
        computeEdgeSmoothness(*e_it);
      }
    }
  }
  graph_->clusters()->addBorderIndicesToClusters();
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::computeBoundaryProperties(
  ClusterPtr c, EdgePtr e)
{
  int window_size = ceil( std::min( sqrt( static_cast<float>(c->size()) ) / e->boundary_pairs[c->id()].size()
                                    + pow(c->getCentroid()(2), 2) + 5.0, 30.0) );
  for (std::list<int>::iterator b_it = e->boundary_pairs[c->id()].begin(); b_it != e->boundary_pairs[c->id()].end(); ++b_it)
  {
    cob_3d_features::OrganizedNormalEstimationHelper::computeSegmentNormal<PointT,PointLabelT>(
      graph_->edges()->getBoundaryPoint(*b_it).normal, *b_it, surface_, labels_, window_size, 1);
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::computeBoundaryProperties(ClusterPtr c)
{
  std::vector<ClusterPtr> adj_list;
  graph_->getAdjacentClusters(c->id(), adj_list);
  //if (adj_list.size()) std::cout << c->size() << " : " << std::endl;
  for (typename std::vector<ClusterPtr>::iterator a_it = adj_list.begin(); a_it != adj_list.end(); ++a_it)
  {
    //std::cout << (*a_it)->size() << ", " << std::endl;
    computeBoundaryProperties(c, graph_->getConnection(c->id(), (*a_it)->id()));
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::computeEdgeSmoothness(EdgePtr e)
{
  int smooth_points = 0;
  std::list<int>::iterator bp_it, bp_end;
  for (boost::tie(bp_it,bp_end) = e->getBoundaryPairs(); bp_it != bp_end; ++bp_it)
  {
    BoundaryPoint bp_this = graph_->edges()->getBoundaryPoint(*bp_it);
    /*float angle = fabs( atan2(bp_this.normal.cross(graph_->edges()->getBoundaryPoint(bp_this.brother).normal).norm(),
      bp_this.normal.dot(graph_->edges()->getBoundaryPoint(bp_this.brother).normal)) );*/
    //if (angle < max_boundary_angle_)
    // No fabs for dot product here, normals are correct aligned
    if (bp_this.normal.dot(graph_->edges()->getBoundaryPoint(bp_this.brother).normal) > min_dot_boundary_)
      ++smooth_points;
  }
  e->smoothness = static_cast<float>(smooth_points) / static_cast<float>(e->size());
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::computeEdgeSmoothness()
{
  EdgePtr e_it, e_end;
  for (boost::tie(e_it,e_end) = graph_->edges()->getEdges(); e_it != e_end; ++e_it) { computeEdgeSmoothness(e_it); }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::getPotentialObjects(
  std::map<int,int>& objs, int max_size) //objs[cluster_id] = object_id
{
  std::set<int> processed;
  ClusterPtr c_it, c_end;
  int obj_counter = 0, prev_size;
  for ( boost::tie(c_it,c_end) = graph_->clusters()->getClusters(); c_it != c_end; ++c_it )
  {
    if (processed.find(c_it->id()) != processed.end()) continue;
    processed.insert(c_it->id());
    if (c_it->size() > max_size) continue;
    objs[c_it->id()] = obj_counter;
    prev_size = objs.size();
    addSmallNeighbors(c_it, objs, processed, obj_counter, max_size);
    if (objs.size() == prev_size) objs.erase(c_it->id());
    else ++obj_counter;
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::addSmallNeighbors(
  ClusterPtr c, std::map<int,int>& objs, std::set<int>& processed, int obj_counter, int max_size)
{
  std::vector<ClusterPtr> c_adj;
  graph_->getAdjacentClusters(c->id(),c_adj);
  for (typename std::vector<ClusterPtr>::iterator it=c_adj.begin(); it!=c_adj.end(); ++it)
  {
    if (processed.find((*it)->id()) != processed.end()) continue;
    processed.insert((*it)->id());
    if ((*it)->size() > max_size) continue;

    objs[(*it)->id()] = obj_counter;
    addSmallNeighbors(*it, objs, processed, obj_counter, max_size);
  }
  return;
}

#endif
