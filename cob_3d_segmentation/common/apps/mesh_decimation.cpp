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
 * \date Date of creation: 06/2013
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

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/surface/organized_fast_mesh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <cob_3d_mapping_filters/downsample_filter.h>
#include <cob_3d_mapping_common/stop_watch.h>

std::string file_in_;
std::string file_out_;

int readOptions(int argc, char** argv)
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help,h", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input folder with data points")
    ("out,o", value<std::string>(&file_out_), "out file with data points")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out",1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if(vm.count("help") || argc == 1)
  { std::cout << options << std::endl; return(-1); }
}

template<typename PointT>
class MeshSimplification
{
public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  struct V; // vertex
  struct E; // edge
  struct F; // face

  // adjacencs_list< OutEdgeList, VertexList, TransitionType[, VertexProperties, EdgeProperties, GraphProperties, EdgeList] >
  typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS, V, E> GraphT;
  typedef typename boost::graph_traits<GraphT>::vertex_descriptor VertexID;
  typedef typename boost::graph_traits<GraphT>::edge_descriptor EdgeID;
  typedef typename std::list<F>::iterator FacePtr;

  struct V
  {
    V(const Eigen::Vector4f& point) : idx(0), Q(Eigen::Matrix4f::Zeros()) { v = point; }

    Eigen::Vector4f v;
    Eigen::Matrix4f Q;
    int idx;
  };

  struct E
  {
    E() : cost(0) { }

    float cost;
    std::vector<FacePtr> faces;
  };

  struct F
  {
    F() : adjacent_faces(0) { }

    V*& operator[](size_t idx) { return vertices[idx]; }
    V*const operator[](size_t idx) const { return vertices[idx]; }

    V* vertices[3];
    int adjacent_faces;
  };

  struct CompareEdgeCost
  {
    CompareEdgeCost(GraphT* g) { g_ = g; }

    bool operator() (const EdgeID& e1, const EdgeID& e2) const { return (*g_)[e1].cost < (*g_)[e2].cost; }

    GraphT* g_;
  };


public:
  MeshSimplification()
    { }

  inline void setInputCloud(const PointCloudConstPtr& input) { input_ = input; }

  void initializeGraph(const pcl::PolygonMesh& mesh)
    {
      input = new PointCloud;
      pcl::fromROSMsg(mesh.cloud, *input);
      initialzeGraph(mesh.polygons);
    }

  void initializeGraph(const std::vector<pcl::Vertices>& faces)
    {
      std::vector<std::pair<bool,VertexID> > added(input_->size(), std::pair<bool,VertexID>(false,VertexID()));
      for(std::vector<pcl::Vertices>::iterator it = faces.begin(); it != faces.end(); ++it)
      {
        VertexID v[3];
        Eigen::Vector3f p;
        FacePtr f = faces_.insert(faces_.end(), F());

        for(int i=0; i<=2; ++i)
        {
          int idx = it->vertices[i];
          if( added[idx].first ) { v[i] = added[idx].second; }
          else
          {
            p = (*input_)[idx].getVector3fMap();
            v[i] = boost::add_vertex(V(Eigen::Vector4f( p(0), p(1), p(2), 1.0 )), g_);
            added[idx].first = true;
            added[idx].second = v[i];
          }
          (*f)[i] = &g_[v[i]];
        }

        g_[ boost::add_edge(v[0], v[1], g_)->first ].faces.push_back(f);
        g_[ boost::add_edge(v[1], v[2], g_)->first ].faces.push_back(f);
        g_[ boost::add_edge(v[2], v[0], g_)->first ].faces.push_back(f);
      }
    }

  void convertGraph(pcl::PolygonMesh& mesh)
    {
      PointCloud out;
      out.resize(boost::num_vertices(g_));
      int idx = 0;
      typename boost::graph_traits<GraphT>::vertex_iterator v_it, v_end;
      for (boost::tie(v_it,v_end) = boost::vertices(g_); v_it != v_end; ++v_it)
      {
        Eigen::Vector4f p = g_[*v_it].v;
        out[idx].x = p(0);
        out[idx].y = p(1);
        out[idx].z = p(2);
        g_[*v_it].idx = idx;
      }
      pcl::toROSMsg(out, mesh.cloud);
      idx = 0;
      mesh.polygons.resize(faces_.size());
      for (FacePtr f = faces_.begin(); f != faces_.end(); ++f)
      {
        for(int i=0; i<=2; ++i) { mesh.polygons[idx].vertices.push_back( (*f)[i]->idx ); }
        ++idx;
      }
    }

  inline Eigen::Matrix4f computeQ(const pcl::Vertices& face)
    {
    }

private:

  PointCloudConstPtr input_;

  GraphT g_;
  std::priority_queue<EdgeID, std::vector<EdgeID>, CompareEdgeCost> heap_;
  std::list<F> faces_;

};

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  pcl::PCDReader r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_(new pcl::PointCloud<pcl::PointXYZRGB>);
  r.read(file_in_, *input);

  pcl::PolygonMesh tmp_mesh;


  PrecisionStopWatch t;
  t.precisionStart();
  cob_3d_mapping_filters::DownsampleFilter<pcl::PointXYZRGB> down;
  down_->header = input->header;
  down.setInputCloud(input);
  down.filter(*down_);
  //*segmented_ = *down_;
  std::cout << "downsampling took " << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;
  ofm.setInputCloud(down_);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
  ofm.reconstruct(tmp_mesh);
  std::cout << "Fast Mesh took " << t.precisionStop() << "s." << std::endl;

  pcl::visualization::PCLVisualizer v;
  v.addPolygonMesh(tmp_mesh);

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  return 0;
}
