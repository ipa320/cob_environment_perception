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

#ifndef __IMPL_DEPTH_SEGMENTATION_HPP__
#define __IMPL_DEPTH_SEGMENTATION_HPP__

#include <set>

#include <pcl/common/eigen.h>
#include <Eigen/LU>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_features/depth_segmentation.h"

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_mapping_features::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::addIfIsValid(
  int u, int v, int idx, int idx_prev, Eigen::Vector3f& n, 
  std::multiset<SegmentationCandidate>& coords_todo, ClusterPtr c)
{
  int* p_label = &(labels_->points[idx].label);
  if (*p_label == I_UNDEF)
  {
    float angle = fabs( atan2((n.cross(normals_->points[idx].getNormalVector3fMap())).norm(),
			n.dot(normals_->points[idx].getNormalVector3fMap())) );

    if ( (int)c->size() < min_cluster_size_ || angle < max_angle_)
    {
      *p_label = c->id();;
      coords_todo.insert(SegmentationCandidate(u, v, angle));
    }
  }
  else if (*p_label > I_EDGE && c->id() != *p_label)
  {
    graph_->edges()->addBoundaryPair(graph_->connect(c->id(), *p_label), c->id(), idx_prev, *p_label, idx);
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_mapping_features::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::performInitialSegmentation()
{
  int width = labels_->width, height = labels_->height;
  // reset graph and property handlers:
  graph_->clear();
  graph_->clusters()->setLabelCloud(labels_);
  graph_->clusters()->setPointCloud(surface_);
  graph_->clusters()->setNormalCloud(normals_);
  graph_->edges()->setLabelCloud(labels_);
  graph_->edges()->setPointCloud(surface_);
  graph_->clusters()->createCluster(I_NAN)->type = I_NAN;
  graph_->clusters()->createCluster(I_BORDER)->type = I_BORDER;
  graph_->clusters()->createCluster(I_EDGE)->type = I_EDGE;

  // fire!
  for(size_t i=0; i<labels_->points.size(); ++i)
  {
    if (labels_->points[i].label == I_UNDEF)
    {
      ClusterPtr c = graph_->clusters()->createCluster();
      std::multiset<SegmentationCandidate> coords_todo;
      coords_todo.insert(SegmentationCandidate(i%width, i/width, 0.0));
      labels_->points[i].label = c->id();
      while (coords_todo.size() != 0)
      {
	SegmentationCandidate p = *coords_todo.begin();
	coords_todo.erase(coords_todo.begin());
	float angle;
	Eigen::Vector3f p_i;
	int p_idx = p.v * width + p.u;
	graph_->clusters()->addPoint(c, p_idx); // clusterhandler takes care of properties
 
	Eigen::Vector3f n_n = c->getOrientation();
	// Look right
	if (p.u+1 < width) { addIfIsValid(p.u+1, p.v, p_idx+1, p_idx, n_n, coords_todo, c); }
	// Look down
	if (p.v+1 < height) { addIfIsValid(p.u, p.v+1, p_idx+width, p_idx, n_n, coords_todo, c); }
	// Look left
	if (p.u > 0) { addIfIsValid(p.u-1, p.v, p_idx-1, p_idx, n_n, coords_todo, c); }
	// Look up
	if (p.v > 0) { addIfIsValid(p.u, p.v-1, p_idx-width, p_idx, n_n, coords_todo, c); }
      } // end while
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
cob_3d_mapping_features::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::refineSegmentation()
{
  graph_->clusters()->sortBySize(); // Ascending order
  std::cout << "Clusters: " << graph_->clusters()->numClusters() << std::endl;;
  ClusterPtr c_it, c_end;
  std::cout << "BoundaryProperites" << std::endl;
  int num_p = 0, num_c = 0;
  for (boost::tie(c_it,c_end) = graph_->clusters()->getClusters(); c_it != c_end; ++c_it)
  {
    num_p += c_it->size();
    if (c_it->size() > 20) ++num_c;
  }
  std::cout << "Points: " << num_p << " C: " << num_c << std::endl;
  
  for (boost::tie(c_it,c_end) = graph_->clusters()->getClusters(); c_it != c_end; ++c_it)
  {
    //std::cout << c_it->size() << std::endl;
    if (c_it->size() < 20) continue;
    if (c_it->type == I_EDGE || c_it->type == I_NAN || c_it->type == I_BORDER) continue;
    computeBoundaryProperties(c_it);
  }
  std::cout << "Edge Smoothness" << std::endl;
  computeEdgeSmoothness();
  for ( --c_end; c_end != graph_->clusters()->begin(); --c_end) // iterate from back to front
  {
    if ( c_end->size() < 5 ) break;
    if ( c_end->type == I_EDGE || c_end->type == I_NAN || c_end->type == I_BORDER) continue;
    std::vector<ClusterPtr> adj_list;
    std::cout << "Merge: " << c_end->id() << "(" << c_end->size() << ")" << std::endl;
    graph_->getConnectedClusters(c_end->id(), adj_list, graph_->edges()->edge_validator);
    std::cout << "Connected: " << adj_list.size() << std::endl;
    for (typename std::vector<ClusterPtr>::iterator a_it = adj_list.begin(); a_it != adj_list.end(); ++a_it)
    {
      graph_->merge( (*a_it)->id(), c_end->id() );
    }
    std::cout << "Components..." << std::endl;
    graph_->clusters()->computeClusterComponents(c_end);
    std::cout << "Is Plane: " << (c_end->is_save_plane ? "True" : "False" ) << std::endl;
    if (!c_end->is_save_plane)
    {
      //graph_->clusters()->recomputeClusterNormals(c_end);
      graph_->clusters()->computeCurvature(c_end);
      if (c_end->max_curvature < 0.02) 
	c_end->type = I_PLANE;
      else if (c_end->max_curvature < 9.0 * c_end->min_curvature) 
	c_end->type = I_SPHERE;
      else
	c_end->type = I_CYL;
    }
    else c_end->type = I_PLANE;
    std::cout << "Done!" << std::endl;
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_mapping_features::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::computeBoundaryProperties(ClusterPtr c)
{
  std::vector<ClusterPtr> adj_list;
  graph_->getAdjacentClusters(c->id(), adj_list);
  int perimeter = 0;
  for (typename std::vector<ClusterPtr>::iterator a_it = adj_list.begin(); a_it != adj_list.end(); ++a_it)
  {
    EdgePtr e = graph_->getConnection(c->id(), (*a_it)->id());
    if (e->boundary_pairs.find(c->id()) == e->boundary_pairs.end()) std::cout << "not there" << std::endl;
    perimeter = ceil( std::min( sqrt( static_cast<float>(c->size()) ) / e->boundary_pairs[c->id()].size() 
				+ pow(c->getCentroid()(2), 2) + 5.0, 30.0) );
    for (std::list<int>::iterator b_it = e->boundary_pairs[c->id()].begin(); b_it != e->boundary_pairs[c->id()].end(); ++b_it)
    {
      graph_->edges()->computeBoundaryPointProperties(perimeter, *b_it, graph_->edges()->getBoundaryPoint(*b_it));
    }
  }
}

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_mapping_features::DepthSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::computeEdgeSmoothness()
{
  EdgePtr e_it, e_end;
  std::list<int>::iterator bp_it, bp_end;
  for (boost::tie(e_it,e_end) = graph_->edges()->getEdges(); e_it != e_end; ++e_it)
  {
    int smooth_points = 0;
    for (boost::tie(bp_it,bp_end) = e_it->getBoundaryPairs(); bp_it != bp_end; ++bp_it)
    {
      BoundaryPoint bp_this = graph_->edges()->getBoundaryPoint(*bp_it);
      if (max_boundary_angle_ < bp_this.normal.dot(graph_->edges()->getBoundaryPoint(bp_this.brother).normal))
	++smooth_points;
    }
    e_it->smoothness = static_cast<float>(smooth_points) / static_cast<float>(e_it->size());
  }
}

#endif
