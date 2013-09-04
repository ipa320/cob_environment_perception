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

#ifndef COB_MESH_TYPES_H
#define COB_MESH_TYPES_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace cob_3d_meshing
{
  namespace MeshProperties
  {
    template<typename PointT>
    class Points
    {
    public:
      typedef PropertyTraits<Points<PointT> >::MeshT MeshT;
      typedef PropertyTraits<Points<PointT> >::VertexHandle VertexHandle;

      void init(MeshT* mesh_hdl) { return; }
      void update(const VertexHandle& vertex_handle, int idx) { return; }

    };

    template<typename PointNormalT>
    class Normals
    {
    public:
      typedef PropertyTraits<Normals<PointNormalT> >::MeshT MeshT;
      typedef PropertyTraits<Normals<PointNormalT> >::VertexHandle VertexHandle;

      typedef typename pcl::PointCloud<PointNormalT>::ConstPtr CloudPtr;
      typedef OpenMesh::Vec3f PropType;
      typedef OpenMesh::VPropHandleT<PropType> PropHandle;

      Normal(const CloudPtr& normals){ }

      void init(MeshT* mesh_hdl)
      {
        mesh_hdl->add_property(prop_hdl_, "Normals");
      }

      void update(const VertexHandle& vertex_hdl, int idx)
      {
        Eigen::Vector3f n = (*cloud_)[idx].getNormalVector3fMap();
        mesh_hdl->property(prop_hdl_, vertex_hdl) = PropType(n(0), n(1), n(2));
      }

    private:
      CloudPtr cloud_;
      PropHandle prop_hdl_;
    };

    template<typename PropT> struct PropertyTraits;

    template<>
      struct PropertyTraits<Points>
    {
      typedef OpenMesh::TriMesh_ArrayKernelT<> MeshT;
      typedef MeshT::VertexHandle VertexHandle;

      static std::string name() { return "Points"; }
    };


    template<>
    template<typename PointNormalT>
      struct PropertyTraits<Normals<PointNormalT> >
    {
      typedef OpenMesh::TriMesh_ArrayKernelT<> MeshT;
      typedef MeshT::VertexHandle VertexHandle;

      static std::string name() { return "Normals"; }
    };
  }


template<typename PropT>
class Mesh
{
public:
  typedef typename MeshProperties::PropertyTraits<PropT>::MeshT MeshT;

public:
  Mesh(const PropT& properties)
    : mesh_()
  {
    prop_handler_ = properties;
    prop_handler_.init(&mesh_);
  }

  ~Mesh() {}


private:
  MeshT mesh_;
  PropT prop_handler_;
};

}

#endif
