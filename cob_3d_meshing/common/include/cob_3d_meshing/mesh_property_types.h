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

#ifndef COB_MESH_PROPERTY_TYPES_H
#define COB_MESH_PROPERTY_TYPES_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace cob_3d_meshing
{
  class DefaultMeshTraits
  {
  public:
    typedef OpenMesh::TriMesh_ArrayKernelT<> MeshT;
    typedef MeshT::VertexHandle VertexHandle;
    typedef MeshT::FaceHandle FaceHandle;
  };


  namespace MeshProperties
  {
    //______________________________ Normals __________________________________
    template<typename PointNormalT,
             typename MeshTraits = DefaultMeshTraits>
    class Normals
    {
    public:
      typedef PointNormalT T1;
      typedef typename MeshTraits::MeshT MeshT;
      typedef typename MeshTraits::VertexHandle VertexHandle;
      typedef typename pcl::PointCloud<T1>::ConstPtr DataPtrT;
      typedef OpenMesh::Vec3f PropT;
      typedef OpenMesh::VPropHandleT<PropT> PropHandle;

      Normals(const DataPtrT& data) { data_ = data; }

      inline static std::string name() { return "Normals"; }

      inline void init(MeshT* mesh_hdl)
      {
        mesh_ = mesh_hdl;
        mesh_->add_property(prop_hdl_, name());
      }

      inline void setVertex(const VertexHandle& vertex_hdl, int idx)
      {
        Eigen::Vector3f n = (*data_)[idx].getNormalVector3fMap();
        mesh_->property(prop_hdl_, vertex_hdl) = PropT(n(0), n(1), n(2));
      }

    private:
      MeshT* mesh_;
      DataPtrT data_;
      PropHandle prop_hdl_;
    };

    //______________________________ Labels __________________________________
    template<typename PointLabelT,
             typename MeshTraits = DefaultMeshTraits>
    class Labels
    {
    public:
      typedef PointLabelT T1;
      typedef typename MeshTraits::MeshT MeshT;
      typedef typename MeshTraits::VertexHandle VertexHandle;
      typedef typename pcl::PointCloud<T1>::ConstPtr DataPtrT;
      typedef int PropT;
      typedef OpenMesh::VPropHandleT<PropT> PropHandle;

      Labels(const DataPtrT& data) { data_ = data; }

      static std::string name() { return "Labels"; }

      inline void init(MeshT* mesh_hdl)
      {
        mesh_ = mesh_hdl;
        mesh_->add_property( prop_hdl_, name() );
      }

      inline void setVertex(const VertexHandle& vertex_hdl, int idx)
      {
        mesh_->property( prop_hdl_, vertex_hdl ) = (*data_)[idx].label;
      }

    private:
      MeshT* mesh_;
      DataPtrT data_;
      PropHandle prop_hdl_;
    };

    //______________________________ Color ____________________________________

    //______________________________ PlaneCoef ________________________________
  }
}

#endif
