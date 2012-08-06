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

#ifndef __IMPL_CLUSTER_GRAPH_STRUCTURE_HPP__
#define __IMPL_CLUSTER_GRAPH_STRUCTURE_HPP__

// from cluster.h: #include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_segmentation/cluster_graph_structure.h"

#include <set>

template <typename ClusterHandlerT, typename EdgeHandlerT> void
cob_3d_segmentation::ClusterGraphStructure<ClusterHandlerT,EdgeHandlerT>::merge(const int cid_source, const int cid_target)
{
  std::vector<EdgePtr> updated_edges;
  merge(vid_.find(cid_source)->second, vid_.find(cid_target)->second, updated_edges);
}

template <typename ClusterHandlerT, typename EdgeHandlerT> void
cob_3d_segmentation::ClusterGraphStructure<ClusterHandlerT,EdgeHandlerT>::merge(
  const int cid_source,
  const int cid_target,
  std::vector<EdgePtr>& updated_edges)
{
  merge(vid_.find(cid_source)->second, vid_.find(cid_target)->second, updated_edges);
}

template <typename ClusterHandlerT, typename EdgeHandlerT> void
cob_3d_segmentation::ClusterGraphStructure<ClusterHandlerT,EdgeHandlerT>::merge(
  const VertexID src,
  const VertexID trg,
  std::vector<EdgePtr>& updated_edges)
{
  e_hdl_->erase(g_[boost::edge(src, trg, g_).first].e_it);
  boost::remove_edge(src, trg, g_);
  typename boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
  EdgeID eid;
  bool ok;
  // iterate edges of source and relocate them
  for (boost::tie(oe_it,oe_end) = boost::out_edges(src,g_); oe_it != oe_end; ++oe_it)
  {
    boost::tie(eid,ok) = boost::add_edge(trg, boost::target(*oe_it,g_), g_); // try add new edge
    if (ok) // if not already existing, move EdgePtr
    { 
      g_[eid].e_it = g_[*oe_it].e_it;
      e_hdl_->move(g_[src].c_it->id(), g_[trg].c_it->id(), g_[eid].e_it);
    } 
    else // else use EdgeHandler to merge to target and drop from Handler
    { 
      e_hdl_->merge(g_[*oe_it].e_it, g_[eid].e_it);
    }
    updated_edges.push_back(g_[eid].e_it);
  }
  vid_.erase(g_[src].c_it->id()); // erase from map
  c_hdl_->merge(g_[src].c_it, g_[trg].c_it); // use ClusterHandler to merge properties and drop from handler
  boost::clear_vertex(src, g_); // removes all out edges, but vertex still exists
  boost::remove_vertex(src, g_); // removes vertex, assumes there are no edges left
}


template <typename ClusterHandlerT, typename EdgeHandlerT> void
cob_3d_segmentation::ClusterGraphStructure<ClusterHandlerT,EdgeHandlerT>::getAdjacentClusters(
  int cid, std::vector<ClusterPtr>& adjacent_clusters)
{
  adjacent_clusters.clear();
  if (vid_.find(cid) == vid_.end()) return;
  typename boost::graph_traits<GraphT>::adjacency_iterator aj_it, aj_end;
  //std::cout << "EDGES: " << boost::out_degree(to_vID_[cID],g_) << std::endl;
  for (boost::tie(aj_it,aj_end) = boost::adjacent_vertices(vid_.find(cid)->second,g_); aj_it != aj_end; ++aj_it)
    adjacent_clusters.push_back(g_[*aj_it].c_it);
}

template <typename ClusterHandlerT, typename EdgeHandlerT> void
cob_3d_segmentation::ClusterGraphStructure<ClusterHandlerT,EdgeHandlerT>::getConnectedClusters(
  int cid_start, std::vector<ClusterPtr>& connected_clusters, boost::function<bool (EdgePtr)> f)
{
  connected_clusters.clear();
  if (vid_.find(cid_start) == vid_.end()) return;
  VertexID curr_c;
  std::set<int> id_set;
  id_set.insert(cid_start);
  std::list<VertexID> vertex_todo;
  vertex_todo.push_back(vid_.find(cid_start)->second);

  while (vertex_todo.size() != 0)
  {
    typename boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
    curr_c = vertex_todo.front();
    vertex_todo.pop_front();
    //std::cout << g_[curr_c].c_it->size() << std::endl;
    //std::cout << boost::out_degree(curr_c,g_) << std::endl;
    for (boost::tie(oe_it,oe_end) = boost::out_edges(curr_c,g_); oe_it != oe_end; ++oe_it)
    {
      VertexID new_id = boost::target(*oe_it,g_);
      if ( id_set.find(g_[new_id].c_it->id()) == id_set.end() && f(g_[*oe_it].e_it) )
      {
	vertex_todo.push_back(new_id);
	id_set.insert(g_[new_id].c_it->id());
	connected_clusters.push_back(g_[new_id].c_it);
      }
    }
  }
  //std::cout << "search done" << std::endl;
}

/*
void
cob_3d_segmentation::ClusterList::computeEdgeAngles(int cID)
{
  boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
  Eigen::Vector3f c_n = g_[to_vID_[cID]].c_it->getOrientation();
  for (boost::tie(oe_it, oe_end) = boost::out_edges(to_vID_[cID],g_); oe_it != oe_end; ++oe_it)
  {
    Eigen::Vector3f a_n = g_[boost::target(*oe_it,g_)].c_it->getOrientation();
    g_[*oe_it].angle = atan2 ( (a_n.cross(c_n)).norm(), a_n.dot(c_n) );
    g_[*oe_it].d_size = abs(g_[to_vID_[cID]].c_it->size() - g_[boost::target(*oe_it,g_)].c_it->size());
  }
}

void
cob_3d_segmentation::ClusterList::computeEdgeSmoothness(const float max_angle)
{
  boost::graph_traits<GraphT>::edge_iterator e_it, e_end;
  for (boost::tie(e_it,e_end) = boost::edges(g_); e_it != e_end; ++e_it)
  {
    std::map<int,int>::iterator bp_it = g_[*e_it].boundary_points.begin()->second.begin();
    int smooth_points = 0;
    for ( ; bp_it != g_[*e_it].boundary_points.begin()->second.end(); ++bp_it)
    {
      if (max_angle < boundary_points_[bp_it->first].normal.dot(boundary_points_[bp_it->second].normal))
	++smooth_points;
    }
    g_[*e_it].smoothness = static_cast<float>(smooth_points) / static_cast<float>(g_[*e_it].boundary_points.begin()->second.size());
  }
}
*/

/* not working
void
cob_3d_segmentation::ClusterGraphStructure::removeSmallClusters()
{
  boost::graph_traits<GraphT>::vertex_iterator v_it, v_del, v_end;
  boost::tie(v_it,v_end)=boost::vertices(g_);
  boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
  int size;
  while( v_it != v_end)
  {
    //std::cout << "ID: " << g_[*v_it].c_it->id << std::endl;
    int sum_borders = 0;
    std::pair<int,VertexID> big_brother(0,*v_it);
    for (boost::tie(oe_it,oe_end)=boost::out_edges(*v_it,g_); oe_it != oe_end; ++oe_it)
    {
      sum_borders += g_[*oe_it].width;
      if ( (size = g_[boost::target(*oe_it,g_)].c_it->size()) > big_brother.first )
	big_brother = std::pair<int,VertexID>(size, boost::target(*oe_it,g_));
    }
    //std::cout << "Borders="<< sum_borders << " Size="<<g_[*v_it].c_it->size()<<" "<<big_brother.first<<std::endl;
    
    if ( ((size = g_[*v_it].c_it->size()) < 20 && size > 0 && big_brother.first > 0) || 2 * sum_borders > size)
    {
      v_del = v_it++;
      mergeClusterDataStructure(*v_del, big_brother.second);
    }
    else ++v_it;

    //std::cout << "... done" << std::endl;
  }
}
*/

/* not working
void
cob_3d_segmentation::ClusterList::getMinAngleAdjacentClusters(
  int cID_start, 
  const float max_angle, 
  std::vector<ClusterPtr>& adjacent_clusters)
{
  adjacent_clusters.clear();
  VertexID curr_c = to_vID_[cID_start];
  std::set<int> id_set;
  id_set.insert(cID_start);
  std::list<VertexID> vertex_todo;
  vertex_todo.push_back(to_vID_[cID_start]);
  /* //only follow minimal angle
  bool still_something_todo = true;
  while (still_something_todo)
  {
    still_something_todo = false;
    std::pair<float, int> min_c(std::numeric_limits<float>::max(), 0);
    boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
    for (boost::tie(oe_it,oe_end) = boost::out_edges(curr_c,g_); oe_it != oe_end; ++oe_it)
    {
      float curr_angle = fabs(g_[*oe_it].angle);
      int curr_id = g_[boost::target(*oe_it,g_)].c_it->id;
      if (curr_angle < min_c.first && id_set.find(curr_id) == id_set.end()
	  && curr_angle < max_angle && curr_angle != std::numeric_limits<float>::quiet_NaN()
	  && g_[boost::target(*oe_it,g_)].c_it->indices.size() > g_[*oe_it].width * 10 )
      {
	min_c = std::pair<float, int>(curr_angle, curr_id);
	still_something_todo = true;
      }
    }
    if (still_something_todo)
    {
      id_set.insert(min_c.second);
      curr_c = to_vID_[min_c.second];
      adjacent_clusters.push_back(g_[curr_c].c_it);
    }
  }
  */
/*
  while (vertex_todo.size() != 0)
  {
    boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
    curr_c = vertex_todo.front();
    vertex_todo.pop_front();
    for (boost::tie(oe_it,oe_end) = boost::out_edges(curr_c,g_); oe_it != oe_end; ++oe_it)
    {
      float new_angle = g_[*oe_it].angle;
      std::cout << new_angle << std::endl;
      VertexID new_id = boost::target(*oe_it,g_);
      //fabs(new_angle) < max_angle && 
      if (id_set.find(g_[new_id].c_it->id) == id_set.end()
	  && new_angle != std::numeric_limits<float>::quiet_NaN() 
	  && (fabs(new_angle) *  g_[*oe_it].d_size) < 500)
	    //&& abs(g_[curr_c].c_it->size() - g_[new_id].c_it->size()) < 0.8 * g_[curr_c].c_it->size() )
      {
	vertex_todo.push_back(new_id);
	id_set.insert(g_[new_id].c_it->id);
	adjacent_clusters.push_back(g_[new_id].c_it);
      }
    }
  }
}
*/

/* not working
void
cob_3d_segmentation::ClusterList::getAdjacentClustersWithSmoothBoundaries(
  int cID_start, 
  const float min_smoothness,
  std::vector<ClusterPtr>& adjacent_clusters)
{
  adjacent_clusters.clear();
  VertexID curr_c = to_vID_[cID_start];
  std::set<int> id_set;
  id_set.insert(cID_start);
  std::list<VertexID> vertex_todo;
  vertex_todo.push_back(to_vID_[cID_start]);

  while (vertex_todo.size() != 0)
  {
    boost::graph_traits<GraphT>::out_edge_iterator oe_it, oe_end;
    curr_c = vertex_todo.front();
    vertex_todo.pop_front();
    for (boost::tie(oe_it,oe_end) = boost::out_edges(curr_c,g_); oe_it != oe_end; ++oe_it)
    {
      float smooth = g_[*oe_it].smoothness;
      //std::cout << smooth << std::endl;
      VertexID new_id = boost::target(*oe_it,g_);
      //fabs(new_angle) < max_angle && 
      if (id_set.find(g_[new_id].c_it->id) == id_set.end()
	  && smooth > min_smoothness)
      {
	vertex_todo.push_back(new_id);
	id_set.insert(g_[new_id].c_it->id);
	adjacent_clusters.push_back(g_[new_id].c_it);
      }
    }
  }
}
*/

#endif
