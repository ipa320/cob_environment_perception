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

//== INCLUDES ==================================================================

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <cob_3d_mapping_filters/downsample_filter.h>
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_mapping_common/sensor_model.h>
#include <cob_3d_features/organized_normal_estimation_omp.h>
#include <cob_3d_segmentation/mesh_decimation.h>
#include <cob_3d_segmentation/impl/fast_segmentation.hpp>


//== GLOBAL VARS ===============================================================
std::string file_in_;
std::string file_out_;
bool save_pcl_;
int n_vertices_;


int readOptions(int argc, char** argv)
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help,h", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input folder with data points")
    ("out,o", value<std::string>(&file_out_), "out file with data points")
    ("n_vertices,n", value<int>(&n_vertices_)->default_value(10000),
     "decimate to n vertices")
    ("pcl", "save pcl results")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out",1);
  variables_map vm;
  store(command_line_parser(argc, argv)
        .options(options).positional(p_opt).run(), vm);
  notify(vm);

  if(vm.count("help") || argc == 1)
  { std::cout << options << std::endl; return(-1); }
  if(vm.count("pcl")) save_pcl_ = true;
  else save_pcl_ = false;

  return 0;
}


namespace OpenMesh {
namespace Decimater {

template<typename MeshT>
void ModNormalQuadricT<MeshT>::initialize()
{
  /*
  using OpenMesh::Geometry::Quadricd;
  if (!quadrics_.is_valid())
    Base::mesh().add_property( quadrics_ );

  typename MeshT::VertexIter v_it  = Base::mesh().vertices_begin();
  typename MeshT::VertexIter v_end = Base::mesh().vertices_end();

  for (; v_it!=v_end; ++v_it)
  {
    Vec4f p = Base::mesh().property(normals_, v_it);
    //std::cout << p << std::endl;
    Base::mesh().property(quadrics_, v_it)
      = Quadricd(p[0], p[1], p[2], p[3]);
  }
  */

  using OpenMesh::Geometry::Quadricd;
  if (!quadrics_.is_valid())
    Base::mesh().add_property( quadrics_ );

  typename Mesh::VertexIter v_it  = Base::mesh().vertices_begin();
  typename Mesh::VertexIter v_end = Base::mesh().vertices_end();

  for (; v_it!=v_end; ++v_it)
    Base::mesh().property(quadrics_, v_it).clear();

  typename Mesh::FaceIter f_it  = Base::mesh().faces_begin();
  typename Mesh::FaceIter f_end = Base::mesh().faces_end();
  typename Mesh::FaceVertexIter fv_it;
  typename Mesh::VertexHandle vh0, vh1, vh2;
  Vec4f n;

  for (; f_it!=f_end; ++f_it)
  {
    fv_it = Base::mesh().fv_iter(f_it.handle());
    for (int i=0; i<3; ++i, ++fv_it)
    {
      vh0 = fv_it.handle();
      n = Base::mesh().property(normals_, vh0);
      Base::mesh().property(quadrics_, vh0) += Quadricd(n);
    }

    /*
    vh0 = fv_it.handle(); ++fv_it;
    vh1 = fv_it.handle(); ++fv_it;
    vh2 = fv_it.handle();

    n1 = Base::mesh().property(normals_, vh0);
    n2 = Base::mesh().property(normals_, vh1);
    n3 = Base::mesh().property(normals_, vh2);

    Quadricd q1(n1);
    Quadricd q2(n2);
    Quadricd q3(n3);

    Base::mesh().property(quadrics_, vh0) += q1;
    Base::mesh().property(quadrics_, vh1) += q2;
    Base::mesh().property(quadrics_, vh2) += q3;
    */
  }
}

template<typename MeshT>
void ModNormalQuadricT<MeshT>::set_error_tolerance_factor(double _factor)
{
  if (this->is_binary()) {
    if (_factor >= 0.0 && _factor <= 1.0) {
      // the smaller the factor, the smaller max_err_ gets
      // thus creating a stricter constraint
      // division by error_tolerance_factor_ is for normalization
      double max_err = max_err_ * _factor / this->error_tolerance_factor_;
      set_max_err(max_err);
      this->error_tolerance_factor_ = _factor;

      initialize();
    }
  }
}

}
}


template<typename PointT, typename NormalT, typename LabelT>
void MeshSimplification<PointT,NormalT,LabelT>::initializeMesh(
  const PointCloudConstPtr& input,
  const LabelCloudConstPtr& labels,
  const NormalCloudConstPtr& normals,
  const std::map<int,Eigen::Vector4f>& params)
{
  int rows = input->height - 1; // last row
  int cols = input->width - 1; // last column
  int row_offset;
  // [h]orizontal, [v]ertical, [l]eft, [r]ight edge check
  std::vector<std::vector<bool> > h(rows+1, std::vector<bool>(cols,true));
  std::vector<std::vector<bool> > v(rows, std::vector<bool>(cols+1,true));
  std::vector<std::vector<bool> > l(rows, std::vector<bool>(cols,true));
  std::vector<std::vector<bool> > r(rows, std::vector<bool>(cols,true));
  std::vector<std::vector<Mesh::VertexHandle> >
    vh(rows+1, std::vector<Mesh::VertexHandle>(cols+1)); // vertex handles

  /*
   * +--+--+   p00  h00  p01  h01  p02
   * |  |  |   v00 lr00  v01 lr01  v02
   * +--+--+   p10  h10  p11  h11  p12
   * |  |  |   v10 lr10  v11 lr11  v12
   * +--+--+   p20  h20  p21  h21  p22
   */

  // corners
  h.front().front() = v.front().front() = r.front().front() = false;
  h.front().back()  = v.front().back()  = l.front().back()  = false;
  h.back().front() = v.back().front() = l.back().front() = false;
  h.back().back()  = v.back().back()  = r.back().back()  = false;

  // first and last row
  for(int x = 1; x<cols; ++x)
  {
    h.front()[x-1] = false;
    h.front()[x  ] = false;
    v.front()[x  ] = false;
    l.front()[x-1] = false;
    r.front()[x  ] = false;
    h.back ()[x-1] = false;
    h.back ()[x  ] = false;
    v.back ()[x  ] = false;
    r.back ()[x-1] = false;
    l.back ()[x  ] = false;
  }

  for(int y = 1; y<rows; ++y)
  {
    // left column and right column
    h[y  ].front() = false;
    v[y-1].front() = false;
    v[y  ].front() = false;
    l[y-1].front() = false;
    r[y  ].front() = false;
    h[y  ].back()  = false;
    v[y-1].back()  = false;
    v[y  ].back()  = false;
    r[y-1].back()  = false;
    l[y  ].back()  = false;

    row_offset = y*(cols+1);
    // iterate remaining
    for(int x=1; x<cols; ++x)
    {
      const PointT* p = &(*input)[row_offset+x];
      if( p->z != p->z )
      {
        v[y-1][x  ] = false;
        v[y  ][x  ] = false;
        h[y  ][x-1] = false;
        h[y  ][x  ] = false;
        l[y-1][x  ] = false;
        l[y  ][x-1] = false;
        r[y-1][x-1] = false;
        r[y  ][x  ] = false;
      }
      else
      {
        vh[y][x] = mesh_->add_vertex(Mesh::Point(p->x, p->y, p->z));
        Eigen::Vector4f nd;
        if(params.find((*labels)[row_offset+x].label) != params.end())
          nd = params.find((*labels)[row_offset+x].label)->second;
        else
        {
          Eigen::Vector3f n = (*normals)[row_offset+x].getNormalVector3fMap();
          float d = p->getVector3fMap().dot(n);
          nd = Eigen::Vector4f(n(0),n(1),n(2),d);
        }

        mesh_->property(p_normals_,vh[y][x]) = PropNormalT(nd(0), nd(1), nd(2), nd(3));
        mesh_->property(p_labels_, vh[y][x]) = (*labels)[row_offset+x].label;
      }
    }
  }

  // iterate h and v to check if edge is valid
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pii = input->points.begin();
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pij = pii + 1; // right
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pji = pii + 1 + cols; // below
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pjj = pji + 1; // below right

  for(int y=0; y<rows; ++y)
  {
    for(int x=0; x<cols; ++x)
    {
      // check horizontal and vertical
      if (h[y][x])
        h[y][x] = isNeighbor(pii->getVector3fMap(), pij->getVector3fMap());
      if (v[y][x])
        v[y][x] = isNeighbor(pii->getVector3fMap(), pji->getVector3fMap());

      // check diagonal
      unsigned char status = (l[y][x] << 1) | r[y][x];
      switch(status)
      {
      case 0b00:
        break;
      case 0b01:
        r[y][x] = isNeighbor(pii->getVector3fMap(), pjj->getVector3fMap());
        break;
      case 0b10:
        l[y][x] = isNeighbor(pij->getVector3fMap(), pji->getVector3fMap());
        break;
      case 0b11:
        if( (pij->z - pji->z) > (pii->z - pjj->z) )
        {
          r[y][x] = false;
          l[y][x] = isNeighbor(pij->getVector3fMap(), pji->getVector3fMap());
        }
        else
        {
          l[y][x] = false;
          r[y][x] = isNeighbor(pii->getVector3fMap(), pjj->getVector3fMap());
        }
        break;
      }
      ++pii; ++pij; ++pji; ++pjj;
    }
    // skip the last column
    // note that in the very last iteration, pjj points beyond end()
    ++pii; ++pij; ++pji; ++pjj;
  }

  for(int y=0; y<rows; ++y)
  {
    for(int x=0; x<cols; ++x)
    {
      /* ii-ji-ij | ij-ji-jj | ii-jj-ij | ii-ji-jj
       *  +-+     |    +     |  +-+     |  +
       *  |/      |   /|     |   \|     |  |\
       *  +       |  +-+     |    +     |  +-+     */
      if(l[y][x])
      {
        if (h[y  ][x] && v[y][x  ])
          mesh_->add_face(vh[y][x  ], vh[y+1][x], vh[y  ][x+1]);
        if (h[y+1][x] && v[y][x+1])
          mesh_->add_face(vh[y][x+1], vh[y+1][x], vh[y+1][x+1]);
      }
      else if (r[y][x])
      {
        if (h[y][x] && v[y][x+1])
          mesh_->add_face(vh[y][x], vh[y+1][x+1], vh[y][x+1]);
        if (v[y][x] && h[y+1][x])
          mesh_->add_face(vh[y][x], vh[y+1][x], vh[y+1][x+1]);
      }
    }
  }
}

template<typename PointT, typename NormalT, typename LabelT>
void MeshSimplification<PointT,NormalT,LabelT>::decimate()
{
  dec_->initialize();
  dec_->module(mod_).set_binary(false);
  dec_->decimate_to(n_vertices_);
  mesh_->garbage_collection();
  std::cout << "Vertices: " << dec_->mesh().n_vertices() << "\n"
            << "Faces: " << dec_->mesh().n_faces() << "\n"
            << "Edges: " << dec_->mesh().n_edges() << std::endl;
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    input(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    down(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr
    normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointLabel>::Ptr
    labels(new pcl::PointCloud<PointLabel>);

  cob_3d_features::OrganizedNormalEstimationOMP
    <pcl::PointXYZRGB, pcl::Normal, PointLabel> one;

  cob_3d_segmentation::FastSegmentation
    <pcl::PointXYZRGB, pcl::Normal, PointLabel> seg;

  cob_3d_mapping_filters::DownsampleFilter
    <pcl::PointXYZRGB> dsf;

  MeshSimplification
    <pcl::PointXYZRGB, pcl::Normal, PointLabel> ms;

  pcl::OrganizedFastMesh
    <pcl::PointXYZRGB> ofm;

  pcl::PCDReader r;
  r.read(file_in_, *input);

  PrecisionStopWatch t, tt;

  tt.precisionStart();
  t.precisionStart();
  dsf.setInputCloud(input);
  dsf.filter(*down);
  std::cout << "DownsampleFilter took "
            << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  one.setInputCloud(down);
  one.setOutputLabels(labels);
  one.setPixelSearchRadius(8,2,2);
  one.setSkipDistantPointThreshold(8);
  one.compute(*normals);
  std::cout << "OrganizedNormalEstimationOMP took "
            << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  seg.setInputCloud(down);
  seg.setNormalCloudIn(normals);
  seg.setLabelCloudInOut(labels);
  seg.setSeedMethod(cob_3d_segmentation::SEED_RANDOM);
  seg.compute();

  std::map<int,Eigen::Vector4f> params;

  typedef typename cob_3d_segmentation::FastSegmentation
    <pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterPtr ClusterPtr;
  for (ClusterPtr c = seg.clusters()->begin(); c!=seg.clusters()->end(); ++c)
  {
    if(c->size() < 5) {continue;}
    seg.clusters()->computeClusterComponents(c);
    float d = fabs(c->getCentroid().dot(c->pca_point_comp3));
    params[c->id()] = Eigen::Vector4f(c->pca_point_comp3(0),
                                      c->pca_point_comp3(1),
                                      c->pca_point_comp3(2),
                                      d);
  }
  std::cout << "FastSegmentation took "
            << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  ms.initializeMesh(down, labels, normals, params);
  ms.decimate();
  std::cout << "MeshSimplification took "
            << t.precisionStop() << "s." << std::endl;

  std::cout << "Total Process took "
            << tt.precisionStop() << "s." << std::endl;

  pcl::PolygonMesh pcl_mesh;
  t.precisionStart();
  ofm.setInputCloud(down);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
  ofm.reconstruct(pcl_mesh);
  std::cout << "OrganizedFastMesh took "
            << t.precisionStop() << "s." << std::endl;


  try
  {
    if(save_pcl_) pcl::io::savePLYFile(file_out_, pcl_mesh);
    else
    {
      if ( !OpenMesh::IO::write_mesh(ms.getMesh(), file_out_) )
      {
        std::cerr << "Cannot write mesh to file " << file_out_ << std::endl;
        return 1;
      }
    }
  }
  catch( std::exception& x )
  {
    std::cerr << x.what() << std::endl;
    return 1;
  }

  return 0;
}
