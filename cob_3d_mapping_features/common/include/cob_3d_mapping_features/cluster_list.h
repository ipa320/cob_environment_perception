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
 * ROS package name: cob_3d_mapping_common
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

#ifndef __COB_3D_MAPPING_COMMON_CLUSTER_LIST_H__
#define __COB_3D_MAPPING_COMMON_CLUSTER_LIST_H__

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/properties.hpp>

#include "cob_3d_mapping_features/cluster.h"

namespace cob_3d_mapping_features
{
  typedef std::list<cob_3d_mapping_features::Cluster>::iterator ClusterPtr;

  class ClusterList
  {
    public:
    ClusterList(): g_()
      , clusters_()
      , to_vID_()
      , boundary_points_()
      , max_cID_(0)
    { }

    ~ClusterList()
    { }

    inline int getMaxID() const { return max_cID_; }

    inline void sort() { clusters_.sort(); }

    // allocates space for a new cluster element and returns its reference
    inline ClusterPtr
    addNewCluster(int id = 0)
    {
      // add cluster to list with new ID
      clusters_.push_back(cob_3d_mapping_features::Cluster( (id<=max_cID_ ? ++max_cID_ : max_cID_=id) ));
      // add vertex to graph with pointer to cluster and return its iterator
      return g_[ to_vID_[max_cID_] = boost::add_vertex( Vertex(--clusters_.end()), g_ ) ].c_it;
    }

    // creates edge between two clusters
    inline void
      connect(const int cID_1, const int cID_2, const int idx_1, const int idx_2)
    {
      if (cID_1 == cID_2) return;
      std::pair<EdgeID,bool> res = boost::add_edge(to_vID_[cID_1],to_vID_[cID_2], g_);
      if (!res.second) ++(g_[res.first].width); // if already existing: increment edge width by one
      (g_[res.first].boundary_points[cID_1])[idx_1] = idx_2;
      (g_[res.first].boundary_points[cID_2])[idx_2] = idx_1;
      boundary_points_[idx_1];// = BoundaryPoint();
      boundary_points_[idx_2];// = BoundaryPoint();
    }

    // checks whether both clusters are connected
    inline bool
    areConnected(const int cID_1, const int cID_2)
    { return (boost::edge(to_vID_[cID_1], to_vID_[cID_2], g_)).second; }

    inline Edge&
      getConnection(const int cID_1, const int cID_2)
    { return (g_[boost::edge(to_vID_[cID_1], to_vID_[cID_2], g_).first]); }

    void
    computeEdgeAngles(int cID);

    void
    computeEdgeSmoothness(const float max_angle);

    // merges cluster of cID_source into cID_target and deletes source 
    // (only data structure, properties have to be merge before
    void
    mergeClusterDataStructure(int cID_source, int cID_target);

    void
    removeSmallClusters();

    //
    void
    getAdjacentClusters(int cID, std::vector<ClusterPtr>& adjacent_clusters);

    void
    getMinAngleAdjacentClusters(int cID_start, const float max_angle, std::vector<ClusterPtr>& adjacent_clusters);

    void
    getAdjacentClustersWithSmoothBoundaries(int cID_start, const float min_smoothness,
					    std::vector<ClusterPtr>& adjacent_clusters);



    // boundary point operations:
    inline BoundaryPoint& getBoundaryPoint(int idx) { return boundary_points_[idx]; }

    inline std::pair<std::map<int,int>::iterator, std::map<int,int>::iterator> getBoundaryPointIndices(int cID_1, int cID_2)
    { 
      std::pair<EdgeID,bool> edge = boost::edge(to_vID_[cID_1], to_vID_[cID_2], g_);
      if (edge.second) 
	return std::make_pair(g_[edge.first].boundary_points[cID_1].begin(),g_[edge.first].boundary_points[cID_1].end());
      return std::make_pair(g_[edge.first].boundary_points.begin()->second.end(),
			    g_[edge.first].boundary_points.begin()->second.end());
    }
    
    inline std::map<int,BoundaryPoint>::iterator bp_begin() { return boundary_points_.begin(); }
    inline std::map<int,BoundaryPoint>::iterator bp_end() { return boundary_points_.end(); }
    inline std::size_t bp_size() { return boundary_points_.size(); }
    
    // list std operations:
    inline std::list<Cluster>::iterator operator[](int cID) { return g_[to_vID_[cID]].c_it; }
    inline std::list<Cluster>::iterator begin() { return clusters_.begin(); }
    inline std::list<Cluster>::const_iterator begin() const { return clusters_.begin(); }
    inline std::list<Cluster>::iterator end() { return clusters_.end(); }
    inline std::list<Cluster>::const_iterator end() const { return clusters_.end(); }
    inline std::list<Cluster>::reverse_iterator rbegin() { return clusters_.rbegin(); }
    inline std::list<Cluster>::const_reverse_iterator rbegin() const { return clusters_.rbegin(); }
    inline std::list<Cluster>::reverse_iterator rend() { return clusters_.rend(); }
    inline std::list<Cluster>::const_reverse_iterator rend() const { return clusters_.rend(); }
    inline std::size_t size() const { return clusters_.size(); }
    inline void clear() { boundary_points_.clear(); clusters_.clear(); to_vID_.clear(); g_.clear(); max_cID_ = 0; }


    private:
    struct Vertex 
    {
      Vertex() { };
      Vertex( std::list<Cluster>::iterator it) : c_it(it) { };

      std::list<Cluster>::iterator c_it; 
    };
    
    /*
    enum vertex_info_t { vertex_info };
    namespace boost
    {
      BOOST_INSTALL_PROPERTY(vertex, info);
    }
    */

    // adjacencs_list< OutEdgeList, VertexList, TransitionType[, VertexProperties, EdgeProperties, GraphProperties, EdgeList] >
    typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS, Vertex, Edge> GraphT;
    typedef boost::graph_traits<GraphT>::vertex_descriptor VertexID;
    typedef boost::graph_traits<GraphT>::edge_descriptor EdgeID;

    GraphT g_;
    std::list<Cluster> clusters_;
    std::map<int, VertexID> to_vID_;
    std::map<int, BoundaryPoint> boundary_points_;
    
    int max_cID_;


    void mergeClusterDataStructure(VertexID src, VertexID trg);

  };
}

#endif
