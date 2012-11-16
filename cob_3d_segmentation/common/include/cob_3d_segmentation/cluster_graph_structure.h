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

#ifndef __CLUSTER_GRAPH_STRUCTURE_H__
#define __CLUSTER_GRAPH_STRUCTURE_H__

#include <utility>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "cob_3d_segmentation/cluster_handler.h"
#include "cob_3d_segmentation/edge_handler.h"

namespace cob_3d_segmentation
{
  template <typename ClusterHandlerT, typename EdgeHandlerT>
  class ClusterGraphStructure
  {
  public:
    typedef typename ClusterHandlerT::Ptr ClusterHandlerPtr;
    typedef typename ClusterHandlerT::ClusterType ClusterType;
    typedef typename ClusterHandlerT::ClusterPtr ClusterPtr;
    typedef typename EdgeHandlerT::Ptr EdgeHandlerPtr;
    typedef typename EdgeHandlerT::EdgeType EdgeType;
    typedef typename EdgeHandlerT::EdgePtr EdgePtr;

    typedef boost::shared_ptr<ClusterGraphStructure<ClusterHandlerT, EdgeHandlerT> > Ptr;

  public:
    ClusterGraphStructure()
      : c_hdl_(new ClusterHandlerT)
      , e_hdl_(new EdgeHandlerT)
      , g_()
      , vid_()
    { }

    ClusterGraphStructure(ClusterHandlerPtr c_hdl, EdgeHandlerPtr e_hdl)
      : c_hdl_(c_hdl)
      , e_hdl_(e_hdl)
      , g_()
      , vid_()
    { }

    ~ClusterGraphStructure()
    { }

    inline ClusterHandlerPtr clusters() { return c_hdl_; }
    inline EdgeHandlerPtr edges() { return e_hdl_; }

    inline void clear() { c_hdl_->clear(); e_hdl_->clear(); vid_.clear(); g_.clear(); }

    EdgePtr connect(const int cid1, const int cid2)
    {
      if (cid1 == cid2) { std::cout << "[ClusterGraphStructure::connect]: connect identic IDs" << std::endl; return e_hdl_->end(); }
      if (vid_.find(cid1) == vid_.end()) { vid_[cid1] = boost::add_vertex(GraphVertex(c_hdl_->getCluster(cid1)), g_); }
      if (vid_.find(cid2) == vid_.end()) { vid_[cid2] = boost::add_vertex(GraphVertex(c_hdl_->getCluster(cid2)), g_); }
      std::pair<EdgeID,bool> res = boost::add_edge(vid_[cid1], vid_[cid2], g_);
      if (res.second) g_[res.first].e_it = e_hdl_->createEdge();
      return g_[res.first].e_it;
    }

    inline bool areConnected(const int cid1, const int cid2)
    { return (boost::edge(vid_.find(cid1)->second, vid_.find(cid2)->second, g_)).second; }

    inline EdgePtr getConnection(const int cid1, const int cid2)
    { return g_[boost::edge(vid_.find(cid1)->second, vid_.find(cid2)->second, g_).first].e_it; }

    void merge(const int cid_source, const int cid_target);
    void merge(const int cid_source, const int cid_target, std::vector<EdgePtr>& updated_edges);
    void getAdjacentClusters(int cid, std::vector<ClusterPtr>& adjacent_clusters);
    void getConnectedClusters(int cid_start, std::vector<ClusterPtr>& connected_clusters, boost::function<bool (EdgePtr)> f);


  private:
    struct GraphVertex
    {
      GraphVertex() { };
      GraphVertex(ClusterPtr it) : c_it(it) { };

      ClusterPtr c_it;
    };

    struct GraphEdge
    {
      GraphEdge() {};
      GraphEdge(EdgePtr it) : e_it(it) { };

      EdgePtr e_it;
    };

    // adjacencs_list< OutEdgeList, VertexList, TransitionType[, VertexProperties, EdgeProperties, GraphProperties, EdgeList] >
    typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS, GraphVertex, GraphEdge> GraphT;
    typedef typename boost::graph_traits<GraphT>::vertex_descriptor VertexID;
    typedef typename boost::graph_traits<GraphT>::edge_descriptor EdgeID;

    void merge(const VertexID src, const VertexID trg, std::vector<EdgePtr>& updated_edges);

    ClusterHandlerPtr c_hdl_;
    EdgeHandlerPtr e_hdl_;
    GraphT g_;
    std::map<int, VertexID> vid_;
  };
}

#endif
