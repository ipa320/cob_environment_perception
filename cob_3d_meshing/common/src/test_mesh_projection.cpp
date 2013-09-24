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
 *  ROS package name: cob_3d_meshing
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2013
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

#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_mapping_common/sensor_model.h>

#include <cob_3d_meshing/mesh_types.h>
#include <cob_3d_meshing/mesh_conversion.h>
#include <cob_3d_meshing/mesh_decimation.h>


//== GLOBAL VARS ===============================================================
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
  store(command_line_parser(argc, argv)
        .options(options).positional(p_opt).run(), vm);
  notify(vm);

  if(vm.count("help") || argc == 1)
  { std::cout << options << std::endl; return(-1); }

  return 0;
}

template<typename Vec3T>
inline Eigen::Vector4f makeVector4f(const Vec3T& v, float last=1.0f)
{
  return Eigen::Vector4f(v[0], v[1], v[2], last);
}

template<typename MeshT, typename SensorT = cob_3d_mapping::PrimeSense>
class MeshProjection
{
private:
  typedef Eigen::Vector3f Vec3;
  typedef Eigen::Vector4f Vec4;
  typedef Eigen::Matrix4f Mat4;
  typedef typename SensorT::MatMap MatMap;
  typedef typename MeshT::VertexHandle VertexHandle;
  typedef typename MeshT::FaceHandle FaceHandle;

public:
  void compute(const Mat4& tf_camera, int w, int h,
               const MeshT& mesh,
               std::vector<std::vector<FaceHandle> >& projection)
  {
    Mat4 proj = SensorT::tf_unit_cube() * tf_camera;
    ScanlinePolygonFill<FaceHandle> spf(w,h);

    std::map<VertexHandle, Vec3> transformed;
    typename MeshT::VertexIter v_it = mesh.vertices_begin();
    typename MeshT::VertexIter v_end = mesh.vertices_end();
    for(; v_it != v_end; ++v_it)
    {
      MeshT::Point p = mesh.point(v_it.handle());
      Vec4 v = proj * Vec4( p.x, p.y, p.z, 1.0);
      transformed[v_it.handle()] = (v/v(3)).head<3>();
    }

    typename MeshT::FaceIter f_it = mesh.faces_begin();
    typename MeshT::FaceIter f_end = mesh.faces_end();
    typename MeshT::FaceVertexIter fv_it;
    Vec3 p1, p2, p3, a, b, normal;
    for(; f_it != f_end; ++f_it)
    {
      fv_it = mesh.fv_iter(f_it.handle());
      p1 = transformed[fv_it.handle()]; ++fv_it;
      p2 = transformed[fv_it.handle()]; ++fv_it;
      p3 = transformed[fv_it.handle()];

      a = p2 - p1;
      b = p3 - p1;
      normal = (a.cross(b)).normalized();
      float d = p1.dot(normal);
      spf.addPolygon(f_it.handle(), normal, d);
      spf.addEdge(f_it.handle(), p2(0), p2(1), p1(0), p1(1));
      spf.addEdge(f_it.handle(), p3(0), p3(1), p2(0), p2(1));
      spf.addEdge(f_it.handle(), p1(0), p1(1), p3(0), p3(1));
    }
    spf.fill(projection);
  }
};

int main(int argc, char** argv)
{
  typedef cob_3d_meshing::Mesh<> MeshT;
  typedef pcl::PointXYZRGB PointT;

  if(readOptions(argc, argv)<0) return 0;

  pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>);

  pcl::PCDReader r;
  r.read(file_in_, *input);

  PrecisionStopWatch t;

  t.precisionStart();
  MeshT mesh;
  cob_3d_meshing::MeshConversion<>::fromPointCloud<PointT,MeshT>(input, mesh);
  cob_3d_meshing::MeshDecimation<MeshT>::quadratic(&mesh, 10000);
  std::cout << "MeshSimplification took "
            << t.precisionStop() << "s." << std::endl;

  

  return 0;
}

