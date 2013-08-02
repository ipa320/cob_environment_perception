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

//#include <pcl/geometry/triangle_mesh.h>
//#include <pcl/geometry/mesh_conversion.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/PolygonMesh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


#include <cob_3d_mapping_filters/downsample_filter.h>
#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_mapping_common/sensor_model.h>

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

  return 0;
}

// based on: Surface Simplification Using Quadric Error Metrics, M Garland, P S Heckbert
template<typename PointT>
class MeshSimplification
{
public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh; // Triangle Mesh
  typedef boost::shared_ptr<Mesh> MeshPtr;

  struct V // a Vertex
  {
    V(const Eigen::Vector4f& point) : idx(0), Q(Eigen::Matrix4f::Zero()) { p = point; }

    Eigen::Vector4f p; // x,y,z,1 component of vertex (paper: v)
    Eigen::Matrix4f Q; // Q-Matrix, see Paper
    int idx;
  };

public:
  MeshSimplification()
    { }
  void initializeMesh(const PointCloudPtr& input)
    {
      input_ = input;
      int rows = input_->height - 1; // last row
      int cols = input_->width - 1; // last column
      int row_offset;
      std::vector<std::vector<bool> > h(rows+1, std::vector<bool>(cols,true)); // horizontal edge check
      std::vector<std::vector<bool> > v(rows, std::vector<bool>(cols+1,true)); // vertical edge check
      std::vector<std::vector<bool> > l(rows, std::vector<bool>(cols,true)); // left diagonal edge check
      std::vector<std::vector<bool> > r(rows, std::vector<bool>(cols,true)); // right diagonal edge check

      std::vector<std::vector<Mesh::VertexHandle> > vh(rows+1, std::vector<Mesh::VertexHandle>(cols+1)); // vertex handles
      std::vector<std::vector<Mesh::VertexHandle> > fh;

      /*
       * +--+--+   p00  h00  p01  h01  p02
       * |  |  |   v00 lr00  v01 lr01  v02
       * +--+--+   p10  h10  p11  h11  p12
       * |  |  |   v10 lr10  v11 lr11  v12
       * +--+--+   p20  h20  p21  h21  p22
       */

      mesh_.reset(new Mesh);

      // corners
      h.front().front() = v.front().front() = r.front().front() = false;
      h.front().back()  = v.front().back()  = l.front().back()  = false;
      h.back().front() = v.back().front() = l.back().front() = false;
      h.back().back()  = v.back().back()  = r.back().back()  = false;

      // first and last row
      for(int x = 1; x<cols; ++x)
      {
        h.front()[x-1] = h.front()[x] = v.front()[x] = l.front()[x-1] = r.front()[x] = false;
        h.back()[x-1] = h.back()[x] = v.back()[x] = r.back()[x-1] = l.back()[x] = false;
      }

      for(int y = 1; y<rows; ++y)
      {
        // left column and right column
        h[y].front() = v[y-1].front() = v[y].front() = l[y-1].front() = r[y].front() = false;
        h[y].back() = v[y-1].back() = v[y].back() = r[y-1].back() = l[y].back() = false;

        row_offset = y*cols;
        // iterate remaining
        for(int x=1; x<cols; ++x)
        {
          const PointT* p = &(*input_)[row_offset+x];
          if( p->z != p->z )
            v[y-1][x] = v[y][x] = h[y][x-1] = h[y][x] = l[y-1][x] = l[y][x-1] = r[y-1][x-1] = r[y][x] = false;
          else
            vh[y][x] = mesh_->add_vertex(Mesh::Point(p->x, p->y, p->z));
        }
      }
      // iterate h and v to check if edge is valid
      typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >::const_iterator pii = input_->points.begin();
      typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >::const_iterator pij = pii + 1; // right
      typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >::const_iterator pji = pii + cols; // below
      typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >::const_iterator pjj = pji + 1; // below right
      for(int y=0; y<rows; ++y)
      {
        for(int x=0; x<cols; ++x)
        {
          // check horizontal and vertical
          if(h[y][x]) { h[y][x] = cob_3d_mapping::PrimeSense::areNeighbors(pii->getVector3fMap(), pij->getVector3fMap()); }
          if(v[y][x]) { v[y][x] = cob_3d_mapping::PrimeSense::areNeighbors(pii->getVector3fMap(), pji->getVector3fMap()); }

          // check diagonal
          unsigned char status = (l[y][x] << 1) | r[y][x];
          switch(status)
          {
          case 0b00:
            break;
          case 0b01:
            r[y][x] = cob_3d_mapping::PrimeSense::areNeighbors(pii->getVector3fMap(), pjj->getVector3fMap());
            break;
          case 0b10:
            l[y][x] = cob_3d_mapping::PrimeSense::areNeighbors(pij->getVector3fMap(), pji->getVector3fMap());
            break;
          case 0b11:
            if( (pij->z - pji->z) > (pii->z - pjj->z) )
            {
              r[y][x] = false;
              l[y][x] = cob_3d_mapping::PrimeSense::areNeighbors(pij->getVector3fMap(), pji->getVector3fMap());
            }
            else
            {
              l[y][x] = false;
              r[y][x] = cob_3d_mapping::PrimeSense::areNeighbors(pii->getVector3fMap(), pjj->getVector3fMap());
            }
            break;
          }
          ++pii; ++pij; ++pji; ++pjj;
        }
        ++pii; ++pij; ++pji; ++pjj; // note that in the very last iteration, pjj points beyond end()
      }

      for(int y=0; y<rows; ++y)
      {
        for(int x=0; x<cols; ++x)
        {
          if(l[y][x])
          {
            /*  +-+  ii - ji - ij
             *  |/
             *  +    */
            if(h[y][x] && v[y][x])
            {
              fh.push_back(std::vector<Mesh::VertexHandle>());
              fh.back().push_back(vh[y][x]); fh.back().push_back(vh[y+1][x]); fh.back().push_back(vh[y][x+1]);
            }//mesh_->add_face(vh[y][x], vh[y+1][x], vh[y][x+1]);
            /*    +  ij - ji - jj
             *   /|
             *  +-+   */
            if(h[y+1][x] && v[y][x+1])
            {
              fh.push_back(std::vector<Mesh::VertexHandle>());
              fh.back().push_back(vh[y][x]); fh.back().push_back(vh[y+1][x]); fh.back().push_back(vh[y][x+1]);
            }//mesh_->add_face(vh[y][x+1], vh[y+1][x], vh[y+1][x+1]);
          }
          else if(r[y][x])
          {
            /*  +-+  ii - jj - ij
             *   \|
             *    +  */
            if(h[y][x] && v[y][x+1])
            {
              fh.push_back(std::vector<Mesh::VertexHandle>());
              fh.back().push_back(vh[y][x]); fh.back().push_back(vh[y+1][x]); fh.back().push_back(vh[y][x+1]);
            }//mesh_->add_face(vh[y][x], vh[y+1][x+1], vh[y][x+1]);
            /*  +  ii - ji - jj
             *  |\
             *  +-+  */
            if(v[y][x] && h[y+1][x])
            {
              fh.push_back(std::vector<Mesh::VertexHandle>());
              fh.back().push_back(vh[y][x]); fh.back().push_back(vh[y+1][x]); fh.back().push_back(vh[y][x+1]);
            }//mesh_->add_face(vh[y][x], vh[y+1][x], vh[y+1][x+1]);
          }
        }
      }
    }

/*
        // compute plane coefficents of current face and update all 3 vertices
        // Q_v = sum_overall_faces_adjacent_to_v(coef * coef.transpose())
        Eigen::Vector3f n = (getVertex(*f,2).p - getVertex(*f,1).p).cross(getVertex(*f,3).p - getVertex(*f,1).p);
        n = n.normalize();
        float d = (n.transpose() * getVertex(*f,1).p * n).norm();
        Eigen::Matrix4f q_tmp = Eigen::Vector4f(n(0), n(1), n(2), d) * Eigen::Vector4f(n(0), n(1), n(2), d).transpose();
        for( i=0; i<=2; ++i) { getVertex(*f, i).Q += q_tmp; }
*/

  inline MeshPtr getMesh() { return mesh_; }

private:

  PointCloudConstPtr input_;
  MeshPtr mesh_;

  int max_iteration_;
  float max_cost_;

};

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  pcl::PCDReader r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_(new pcl::PointCloud<pcl::PointXYZRGB>);
  r.read(file_in_, *input);


  PrecisionStopWatch t;
  t.precisionStart();
  cob_3d_mapping_filters::DownsampleFilter<pcl::PointXYZRGB> down;
  down_->header = input->header;
  down.setInputCloud(input);
  down.filter(*down_);
  //*segmented_ = *down_;
  std::cout << "downsampling took " << t.precisionStop() << "s." << std::endl;


  pcl::PolygonMesh pcl_mesh;
  t.precisionStart();
  pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;
  ofm.setInputCloud(down_);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
  ofm.reconstruct(pcl_mesh);
  std::cout << "OrganizedFastMesh took " << t.precisionStop() << "s." << std::endl;

  t.precisionStart();
  MeshSimplification<pcl::PointXYZRGB> ms;
  ms.initializeMesh(down_);
  std::cout << "My mesh initialization took " << t.precisionStop() << "s." << std::endl;

  try
  {
    if ( !OpenMesh::IO::write_mesh(*(ms.getMesh()), file_out_) )
    {
      std::cerr << "Cannot write mesh to file " << file_out_ << std::endl;
      return 1;
    }
  }
  catch( std::exception& x )
  {
    std::cerr << x.what() << std::endl;
    return 1;
  }

  return 0;
}
